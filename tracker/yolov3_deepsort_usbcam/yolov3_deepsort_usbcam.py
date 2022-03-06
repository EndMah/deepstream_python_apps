#!/usr/bin/env python3

################################################################################
# SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import os
import sys
import signal
sys.path.append('../')
import platform
import configparser
import cv2
import pyds
import time
import gi
import shutil
import psutil
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst, GLib, GstBase
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.FPS import GETFPS

# Experiment configs
# number of objects in real time experiment
num_obj=2
# Experiment runtime in seconds 
runtime=15
# camera resolution
width=1280
height=720

timestr = time.strftime("%Y%m%d-%H%M%S")
PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3
past_tracking_meta=[0]
fps_streams={}
fpsarray=[]
motarray=[]
cpuarray=[]
memarray=[]
swaparray=[]
utilsarray=[]

# MOTA values, with mme calculated manually from video
miss=0
fp=0
gt=0

def osd_sink_pad_buffer_probe(pad,info,u_data):
    frame_number=0
    global miss
    global fp
    global gt
    global num_obj
    #Intializing object counter with 0.
    obj_counter = {
        PGIE_CLASS_ID_VEHICLE:0,
        PGIE_CLASS_ID_PERSON:0,
        PGIE_CLASS_ID_BICYCLE:0,
        PGIE_CLASS_ID_ROADSIGN:0
    }
    num_rects=0
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        f=open("track.txt","w+")
        mota=open("mota.txt","w+")
        txtutils=open("utils.txt","w+")
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number=frame_meta.frame_num+1
        num_rects = frame_meta.num_obj_meta
        l_obj=frame_meta.obj_meta_list
        
        # gathering MOTA values, with mme calculated manually from video
        # total gt
        gt+=num_obj
        # total miss
        if num_rects<num_obj:
            miss+=1
        # total fp
        if num_rects>num_obj:
            fp+=1
        motavalues="miss="+str(miss)+"\nfp="+str(fp)+"\ngt="+str(gt)
        mota.write(motavalues)    
        while l_obj is not None:
            try:
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            obj_counter[obj_meta.class_id] += 1
            class_id = obj_meta.class_id
            uniqueId = obj_meta.object_id
            left = round(obj_meta.rect_params.left)
            top = round(obj_meta.rect_params.top)
            width = round(obj_meta.rect_params.width)
            height = round(obj_meta.rect_params.height)
            confidence = round(obj_meta.confidence,2)
            centroid_x=int(round(left+(width/2)))
            centroid_y=int(round(top+(height/2)))
            print(f'ID:{uniqueId} centroid:{centroid_x},{centroid_y} confidence:{confidence}')
            
            # saving to text file: frame_number,ID,centroid_x,centroid_y,confidence,numberofObjects
            motarray.append(str(frame_number)+","+str(uniqueId)+","+str(centroid_x)+","+str(centroid_y)+","+str(confidence)+","+str(num_rects))
            motarray.append("\n")
            
            try: 
                l_obj=l_obj.next
            except StopIteration:
                break
        f.writelines(motarray)
        f.close()
        mota.close()
        
        
        # Acquiring a display meta object. The memory ownership remains in
        # the C code so downstream plugins can still access it. Otherwise
        # the garbage collector will claim it when this probe function exits.
        display_meta=pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        
        # FPS Counter
        # Get frame rate through this probe
        fps = fps_streams["stream{0}".format(frame_meta.pad_index)].calc_fps()
        fpsarray.append(fps)
        fps = "%.1f"%(fps)
        avgfps = "%.1f"%(sum(fpsarray)/len(fpsarray))
        # cpu usage counter
        cpu = psutil.cpu_percent()
        cpuarray.append(cpu)
        avgcpu = "%.1f"%(sum(cpuarray)/len(cpuarray))
        # memory usage counter
        memory = psutil.virtual_memory().percent
        memarray.append(memory)
        avgmem = "%.1f"%(sum(memarray)/len(memarray))
        # swap usage counter
        swap = psutil.swap_memory().percent
        swaparray.append(swap)
        avgswap = "%.1f"%(sum(swaparray)/len(swaparray))
        
        # Setting display text to be shown on screen
        # Note that the pyds module allocates a buffer for the string, and the
        # memory will not be claimed by the garbage collector.
        # Reading the display_text field here will return the C address of the
        # allocated string. Use pyds.get_string() to get the string content.
        py_nvosd_text_params.display_text = "FPS={} Avg FPS={} Frame Number={}\nCPU={}% AvgCPU={}% Memory={}% Avg Memory={}% Swap={}% Avg Swap={}%\nNumber of Objects={} Vehicle_count={} Person_count={}".format(fps, avgfps, frame_number, cpu, avgcpu, memory, avgmem, swap, avgswap, num_rects, obj_counter[PGIE_CLASS_ID_VEHICLE], obj_counter[PGIE_CLASS_ID_PERSON])

        # Now set the offsets where the string should appear
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12

        # Font , font-color and font-size
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 10
        # set(red, green, blue, alpha); set to White
        py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

        # Text background color
        py_nvosd_text_params.set_bg_clr = 1
        # set(red, green, blue, alpha); set to Black
        py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0)
        # Using pyds.get_string() to get display_text as string
        print(pyds.get_string(py_nvosd_text_params.display_text))
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)
        
        txtutils.writelines('AvgFPS='+avgfps+'AvgCPU='+avgcpu+'%\nAvgMem='+avgmem+'%\nAvgSwap='+avgswap+'%')
        txtutils.close()
        try:
            l_frame=l_frame.next
        except StopIteration:
            break
    #past traking meta data
    if(past_tracking_meta[0]==1):
        l_user=batch_meta.batch_user_meta_list
        while l_user is not None:
            try:
                # Note that l_user.data needs a cast to pyds.NvDsUserMeta
                # The casting is done by pyds.NvDsUserMeta.cast()
                # The casting also keeps ownership of the underlying memory
                # in the C code, so the Python garbage collector will leave
                # it alone
                user_meta=pyds.NvDsUserMeta.cast(l_user.data)
            except StopIteration:
                break
            if(user_meta and user_meta.base_meta.meta_type==pyds.NvDsMetaType.NVDS_TRACKER_PAST_FRAME_META):
                try:
                    # Note that user_meta.user_meta_data needs a cast to pyds.NvDsPastFrameObjBatch
                    # The casting is done by pyds.NvDsPastFrameObjBatch.cast()
                    # The casting also keeps ownership of the underlying memory
                    # in the C code, so the Python garbage collector will leave
                    # it alone
                    pPastFrameObjBatch = pyds.NvDsPastFrameObjBatch.cast(user_meta.user_meta_data)
                except StopIteration:
                    break
                for trackobj in pyds.NvDsPastFrameObjBatch.list(pPastFrameObjBatch):
                    print("streamId=",trackobj.streamID)
                    print("surfaceStreamID=",trackobj.surfaceStreamID)
                    for pastframeobj in pyds.NvDsPastFrameObjStream.list(trackobj):
                        print("numobj=",pastframeobj.numObj)
                        print("uniqueId=",pastframeobj.uniqueId)
                        print("classId=",pastframeobj.classId)
                        print("objLabel=",pastframeobj.objLabel)
                        for objlist in pyds.NvDsPastFrameObjList.list(pastframeobj):
                            print('frameNum:', objlist.frameNum)
                            print('tBbox.left:', objlist.tBbox.left)
                            print('tBbox.width:', objlist.tBbox.width)
                            print('tBbox.top:', objlist.tBbox.top)
                            print('tBbox.right:', objlist.tBbox.height)
                            print('confidence:', objlist.confidence)
                            print('age:', objlist.age)
            try:
                l_user=l_user.next
            except StopIteration:
                break
    return Gst.PadProbeReturn.OK	

def mota_values(num_rects):
    mota=open("mota.txt","w+")
    # gathering MOTA values
    # total gt
    gt+=num_obj
    # total miss
    if num_rects<num_obj:
        miss+=1
    # total fp
    if num_rects>num_obj:
        fp+=1
    motavalues="miss="+str(miss)+"\nfp="+str(fp)+"\ngt="+str(gt)
    mota.write(motavalues)
    mota.close()

def quit_loop(loop):
    loop.quit()
    
def main(args):
    # Check input arguments
    if len(args) != 2:
        sys.stderr.write("usage: %s <v4l2-device-path>\n" % args[0])
        sys.exit(1)
    
    if not os.path.exists('results/'):
        os.makedirs('results/')
        
    #get name of script
    script=os.path.basename(args[0])
    script=os.path.splitext(script)[0]
    for i in range(0,len(args)-1):
        fps_streams["stream{0}".format(i)]=GETFPS(i)
    number_sources=len(args)-1
    
    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")

    # Source element for reading from the file
    print("Creating Source \n ")
    source = Gst.ElementFactory.make("v4l2src", "usb-cam-source")
    if not source:
        sys.stderr.write(" Unable to create Source \n")

    caps_v4l2src = Gst.ElementFactory.make("capsfilter", "v4l2src_caps")
    if not caps_v4l2src:
        sys.stderr.write(" Unable to create v4l2src capsfilter \n")

    print("Creating Video Converter \n")

    # Adding videoconvert -> nvvideoconvert as not all
    # raw formats are supported by nvvideoconvert;
    # Say YUYV is unsupported - which is the common
    # raw format for many logi usb cams
    # In case we have a camera with raw format supported in
    # nvvideoconvert, GStreamer plugins' capability negotiation
    # shall be intelligent enough to reduce compute by
    # videoconvert doing passthrough (TODO we need to confirm this)


    # videoconvert to make sure a superset of raw formats are supported
    vidconvsrc = Gst.ElementFactory.make("videoconvert", "convertor_src1")
    if not vidconvsrc:
        sys.stderr.write(" Unable to create videoconvert \n")

    # nvvideoconvert to convert incoming raw buffers to NVMM Mem (NvBufSurface API)
    nvvidconvsrc = Gst.ElementFactory.make("nvvideoconvert", "convertor_src2")
    if not nvvidconvsrc:
        sys.stderr.write(" Unable to create Nvvideoconvert \n")

    caps_vidconvsrc = Gst.ElementFactory.make("capsfilter", "nvmm_caps")
    if not caps_vidconvsrc:
        sys.stderr.write(" Unable to create capsfilter \n")

    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")

    # Use nvinfer to run inferencing on camera's output,
    # behaviour of inferencing is set through config file
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")

    tracker = Gst.ElementFactory.make("nvtracker", "tracker")
    if not tracker:
        sys.stderr.write(" Unable to create tracker \n")

    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")

    # Use convertor to convert from NV12 to RGBA as required by nvosd
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")

    # Create OSD to draw on the converted RGBA buffer
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")

    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")
    
    nvvidconv2 = Gst.ElementFactory.make("nvvideoconvert", "convertor2")
    if not nvvidconv2:
        sys.stderr.write(" Unable to create nvvidconv2 \n")

    capsfilter = Gst.ElementFactory.make("capsfilter", "capsfilter")
    if not capsfilter:
        sys.stderr.write(" Unable to create capsfilter \n")

    caps = Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420")
    capsfilter.set_property("caps", caps)

    encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
    if not encoder:
        sys.stderr.write(" Unable to create encoder \n")
    encoder.set_property("bitrate", 2000000)
    print("Creating Code Parser \n")
    codeparser = Gst.ElementFactory.make("h264parse", "mpeg4-parser")
    if not codeparser:
        sys.stderr.write(" Unable to create code parser \n")

    print("Creating Container \n")
    container = Gst.ElementFactory.make("matroskamux", "matroskamux")
    if not container:
        sys.stderr.write(" Unable to create code parser \n")
    
    print("Creating FILESINK \n")
    sink = Gst.ElementFactory.make("filesink", "filesink")
    if not sink:
        sys.stderr.write(" Unable to create file sink \n")
        
    sink.set_property("location", 'results/'+script+'_'+timestr+'.mkv')
    sink.set_property("sync", 1)
    sink.set_property("async", 0)
    sink.set_property("qos",0)
    
    print("Playing cam %s " %args[1])
    caps_v4l2src.set_property('caps', Gst.Caps.from_string("video/x-raw, height={}".format(height)))
    caps_vidconvsrc.set_property('caps', Gst.Caps.from_string("video/x-raw(memory:NVMM)"))
    source.set_property('device', args[1])
    streammux.set_property('width', width)
    streammux.set_property('height', height)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)
    streammux.set_property('live-source', 1)

    #Set properties of pgie
    pgie.set_property('config-file-path', "pgie_config.txt")

    #Set properties of tracker
    config = configparser.ConfigParser()
    config.read('tracker_config.txt')
    config.sections()

    for key in config['tracker']:
        if key == 'tracker-width' :
            tracker_width = config.getint('tracker', key)
            tracker.set_property('tracker-width', tracker_width)
        if key == 'tracker-height' :
            tracker_height = config.getint('tracker', key)
            tracker.set_property('tracker-height', tracker_height)
        if key == 'gpu-id' :
            tracker_gpu_id = config.getint('tracker', key)
            tracker.set_property('gpu_id', tracker_gpu_id)
        if key == 'll-lib-file' :
            tracker_ll_lib_file = config.get('tracker', key)
            tracker.set_property('ll-lib-file', tracker_ll_lib_file)
        if key == 'll-config-file' :
            tracker_ll_config_file = config.get('tracker', key)
            tracker.set_property('ll-config-file', tracker_ll_config_file)
        if key == 'enable-batch-process' :
            tracker_enable_batch_process = config.getint('tracker', key)
            tracker.set_property('enable_batch_process', tracker_enable_batch_process)
        if key == 'enable-past-frame' :
            tracker_enable_past_frame = config.getint('tracker', key)
            tracker.set_property('enable_past_frame', tracker_enable_past_frame)

    print("Adding elements to Pipeline \n")
    pipeline.add(source)
    pipeline.add(caps_v4l2src)
    pipeline.add(vidconvsrc)
    pipeline.add(nvvidconvsrc)
    pipeline.add(caps_vidconvsrc)
    pipeline.add(streammux)
    pipeline.add(pgie)
    pipeline.add(tracker)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(nvvidconv2)
    pipeline.add(encoder)
    pipeline.add(capsfilter)
    pipeline.add(codeparser)
    pipeline.add(container)
    pipeline.add(sink)

    # we link the elements together
    # v4l2src -> nvvideoconvert -> mux -> 
    # nvinfer -> nvvideoconvert -> nvosd -> video-renderer
    print("Linking elements in the Pipeline \n")
    source.link(caps_v4l2src)
    caps_v4l2src.link(vidconvsrc)
    vidconvsrc.link(nvvidconvsrc)
    nvvidconvsrc.link(caps_vidconvsrc)
    
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        sys.stderr.write(" Unable to get the sink pad of streammux \n")
    srcpad = caps_vidconvsrc.get_static_pad("src")
    if not srcpad:
        sys.stderr.write(" Unable to get source pad of caps_vidconvsrc \n")
    srcpad.link(sinkpad)
    streammux.link(pgie)
    pgie.link(tracker)
    tracker.link(nvvidconv)
    nvvidconv.link(nvosd)
    nvosd.link(nvvidconv2)
    nvvidconv2.link(capsfilter)
    capsfilter.link(encoder)
    encoder.link(codeparser)
    
    sink.get_static_pad("sink").send_event(Gst.Event.new_eos())
    sinkpad1 = container.get_request_pad("video_0")
    if not sinkpad1:
        sys.stderr.write(" Unable to get the sink pad of qtmux \n")
    srcpad1 = codeparser.get_static_pad("src")
    if not srcpad1:
        sys.stderr.write(" Unable to get mpeg4 parse src pad \n")
    srcpad1.link(sinkpad1)
    container.link(sink)

    # create and event loop and feed gstreamer bus mesages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)

    # Lets add probe to get informed of the meta data generated, we add probe to
    # the sink pad of the osd element, since by that time, the buffer would have
    # had got all the metadata.
    osdsinkpad = nvosd.get_static_pad("sink")
    if not osdsinkpad:
        sys.stderr.write(" Unable to get sink pad of nvosd \n")
    osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)


    print("Starting pipeline \n")
    
    # start play back and listed to events
    pipeline.set_state(Gst.State.PLAYING)
    GObject.timeout_add_seconds(runtime, quit_loop, loop)
    try:
      loop.run()
    except:
      pass

    # cleanup
    # sort motchallenge .txt file with the frame number and id
    fn = 'track.txt'
    
    # save txt file the same as script name
    sorted_fn = 'results/'+script+'_'+timestr+'.txt'
    # save utils
    utils = 'results/'+script+'_'+timestr+'_utils.txt'
    shutil.copyfile('utils.txt', utils)

    with open(fn,'r') as first_file:
        rows = first_file.readlines()
        sorted_rows = sorted(rows, key=lambda x: (int(x.split(',')[0]), int(x.split(',')[1])), reverse=False)
        with open(sorted_fn,'w+') as second_file:
            for row in sorted_rows:
                second_file.write(row)
    new_file = os.path.join("results/", script+'_'+timestr+"_MOTA.txt")
    os.rename("mota.txt", new_file)
    os.remove('track.txt')
    os.remove('utils.txt')
    
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    sys.exit(main(sys.argv))

