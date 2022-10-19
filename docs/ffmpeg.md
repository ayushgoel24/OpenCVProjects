- [Available encoders and decoders](#available-encoders-and-decoders)
- [Conversion between codecs](#conversion-between-codecs)
- [Change the output resolution](#change-the-output-resolution)
- [ffmpeg metadata](#ffmpeg-metadata)
  * [Using ffmpeg to copy metadata](#using-ffmpeg-to-copy-metadata)
  * [Image metadata exif](#image-metadata-exif)
- [Extracting key frames](#extracting-key-frames)
- [Extracting scene-changing frames](#extracting-scene-changing-frames)
- [Rotating video](#rotating-video)

# Available encoders and decoders
```
ffmpeg -encoders
ffmpeg -decoders
```

# Conversion between codecs 

When converting audio and video files with ffmpeg, you do not have to specify the input and output formats. The input file format is auto-detected, and the output format is guessed from the file extension.

```
ffmpeg -i input.mp4 output.webm
```



When converting files, use the `-c:v` and `c:a` option to specify the codecs:
```
ffmpeg -i input.mp4 -c:v libvpx -c:a libvorbis output.webm
```


# Change the output resolution
use flag: `-s <width>x<height>`
for example:	
	
```
ffmpeg -i input.mp4 -s 192x168 out%d.jpg
```
	
	
or with the `-vf` option:

```
ffmpeg -i input.mp4 -vf scale=192:168 out%d.jpg
```

Note: The scale filter can also automatically calculate a dimension while preserving the aspect ratio: `scale=320:-1`, or `scale=-1:240`
```
ffmpeg -i input.mp4 -vf scale=192:-1 out%d.jpg
```



# ffmpeg metadata
Extract metadata:

```
ffmpeg -i input.mp4 -f ffmetadata metadata.txt
exiftool input.mp4 >metadata.txt
```


## Using ffmpeg to copy metadata


ffmpeg, by default, makes all metadata from the first input file available, to the output file muxer, for writing. -map_metadata allows to override that, by either pointing to a different input, or by telling ffmpeg to discard input global metadata (value of -1).


use:
`-movflags use_metadata_tags`

For instance:

```
ffmpeg -i $input_file -movflags use_metadata_tags -crf 22 $output_file
```


The `-metadata` option is for manipulating the metadata. If you just want to copy the metadata from an input file to an output file, you should use the `-map_metadata` option:	
	
```	
ffmpeg -i input.mp4 -map_metadata 0 -c copy output.mp4
```
## Image metadata exif
```
sudo apt install exif
exif image.jpg
```


# Extracting key frames


In video compression, they use the so-called IPB frames

 - I frames (Intra picture): a complete picture
 - P frames (predictive picture): p frames stores difference between current frame and previous frame.
 - B frames (Bidirectionally predicted picture): b-frame stores difference between current frame and previous frame and later frame.

To extract I-frames:
```
ffmpeg -skip_frame nokey -i test.mp4 -vsync vfr -frame_pts true out-%03d.jpg
```
- -vsync vfr: discard the unused frames
- -frame_pts true: use the frame index for image names, otherwise, the index starts from 1 and


with scaling:

```
ffmpeg -i input.mp4   -vf "select='eq(pict_type,I)', scale=640:-1"    -vsync vfr -frame_pts true     out-%03d.jpg
```


We can also use the filter syntax to extract keyframes:
```
ffmpeg -i input.mp4   -vf "select='eq(pict_type,I)'" -vsync vfr -frame_pts true     out-%03d.jpg
```

- -vf filter_graph: set video filters





# Extracting scene-changing frames

If we only want to retain enough info from the video, extracting I-frames only may not be enough. The extracted key frames may still exhibit too much information redundancy. For example, if you have a slow-changing video, the difference between a frame and its subsequent frames will be negligible. To further reduce the number of images generated, we can also use scene filter to select frames that are likely to be a scene-changing frame.



```
ffmpeg -i input.mp4 -vf "select=gt(scene\,0.1), scale=640:-1"  -vsync vfr -frame_pts true out%03d.jpg
```


- select: the frame selection filter
- gt: greater than (>)
- scene: the scene change detection score, values in [0-1]. In order to extract suitable number of frames from the video, for videos with fast-changing frames, we should set this value high, and for videos with mostly still frames, we should set this value low (maybe 0.1 or even less depending on the video content).


# Rotating video

```
ffmpeg -i <input.mp4> -vf "transpose=2,transpose=2" <output.mp4>
```

- 0 = 90CounterCLockwise and Vertical Flip (default)
- 1 = 90Clockwise
- 2 = 90CounterClockwise
- 3 = 90Clockwise and Vertical Flip
Use -vf "transpose=2,transpose=2" for 180 degrees.


If you don't want to re-encode your video AND your player can handle rotation metadata you (rotation in the metadata):


```
ffmpeg -i <input.mp4> -map_metadata 0 -metadata:s:v rotate="180" -codec copy <output.mp4>
```



Refs: [1](https://linuxize.com/post/how-to-install-ffmpeg-on-ubuntu-20-04/), [2](https://ffmpeg.org/ffmpeg-filters.html#scdet-1)






