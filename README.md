This is the kernel driver for my tablet's unique display that has no device manufacturer or model information written on it.
Instead of bothering the mainline people to merge my shitty display panel, I opted instead to just use the panel-simple.c driver
to reuse that code for my own display since it works well enough.

To build, just use make. You'll get a vivax-panel.ko driver that you're supposed to insmod or put in your initram.

DKMS support is coming...

