
# POV (Persistence of Vision) Bike Wheel Light 


## Updates:
- Currently got multiple frame switching working. 
- Next work to try would be to try and save frames to the flash chip and read off of it.
- While one frame is being read from, save flash data to other frame.
- Wifi image transfers can happen maybe when bike has been standing still for some time or just use second core most likely.

This is software that runs on an ESP32 and helps it talk to 168 addressable LED's.
The LED's are attached in a plus (+) pattern on the spokes of a bike wheel (90 deg apart)
They update as the wheel spins creating a persistence of vision effect.
At a typical speed of 20 km/h and 2.8 rotations per second and an angular resolution of 1 degree
these lights  update over 1000 times per second to deliver a crisp still image.

A hall effect sensor and magnet are used to determine the right timing.

This runs on a custom PCB i've made that includes an ESP32 with associated power components, flash memory and addressable LED's.

Photos, PCB schematics, blog post and explanation to come!


## Quick rundown on how it works:

The frame data is just RGB values stored in ledArray.h all in progmem. 
This gets loaded into a DMA buffer in setup. 
While running, we have a second DMA buffer that holds only the data of the current LED state or "slice".
Each time we want to update the LED's (360 times per rotation) we copy the next slice from the first buffer into
the second "slice" buffer. Then we transfer that whole slice buffer to the LED's via a SPI transaction.
I've measured that this copying from one buffer to the other happens in like 3 NANO-seconds. 
The spi transaction is as fast as it can get. 
Every time the hall effect sensor passes the magnet, it triggers an interrupt which updates an index telling the rest of the code
we just crossed "zero" and also how fast the last rotation was in order to calculate how often the LED's need to update.

## Animations:

One of the goals of this device was to be able to play a sequence of frames as an animation. The problem is, this doesn't work unless your animation is updating at only the wheel spin rate times four. Which would be anywhere from say 5-12 fps. Not too bad

Why can't we do it faster? Because of the way the POV effect works. In order for us to see a full frame, the light arms need to sweep through one quadrant (45 deg) to "fill in" the whole light. So the frame needs to be displayed for a whole quarter turn before we update with the next frame. 

So yes, we CAN do animations but they will look better the faster you go, but the faster you go the shorter the time spectators can marvel at your cool lights.

I think we'd do this by storing the frames in the external flash memory chip. Then have two frame buffers. As one frame buffer is being read from by the slice buffer, the other frame buffer will be written to from the flash memory. Each frame, they swap roles so nothing is being read and written to at the same time.

The tricky thing is making sure the copying from flash to the frame buffer is fast enough. Maybe we'd do that in the second core? But that'll probably reserved for the wifi handling. I have no idea at this moment.


## Future work:
- Over the air updates. (A real pain to upload code onto a thing that's atatched to a bike.)
- Wifi frame uploads. 
- Better code layout.
- Animation.


![Messenger_creation_646104513979992](https://github.com/AdamMarciniak/LightWheel/assets/17841889/16a3c867-0d8f-48db-b668-52a30466e095)
