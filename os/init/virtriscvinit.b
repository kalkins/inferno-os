implement Init;

include "sys.m";
        sys: Sys;
        print: import sys;
include "sh.m";
        sh: Sh;
include "draw.m";
        draw: Draw;
        Context: import draw;

Bootpreadlen: con 128;

#Command: module { init: fn(ctxt: ref Context, argv: list of string); };

Init: module
{
    init:   fn();
};

init()
{
    sh = load Sh Sh->PATH;
    sys = load Sys Sys->PATH;
    sys->print("init: starting shell\n");

    shell := load Command "/dis/sh.dis";

    sys->bind("#i", "/dev", sys->MREPL);   # draw device
    sys->bind("#m", "/dev", sys->MAFTER);  # mouse device
    sys->bind("#c", "/dev", sys->MAFTER);  # console device
    sys->bind("#u", "/dev", sys->MAFTER);  # usb
    sys->bind("#S", "/dev", sys->MAFTER);  # storage devices

    #sh->system(nil, "wm/wm");
    spawn shell->init(nil, "sh" :: "-i" :: nil);
}
