# BinaryWatch
TODO:
code is complex enough that it should not be contained entirely in one file.
Seperate out by functionality.

    *display functions
    *RTCC functions
    *deep sleep functions
    *timer functions

Debounce function is actually pretty useful.  Make a library out of it.

    *See if the timers are similar enough that a timer struct can be passed
     in to make the function more generic
    *Pass in the port for the button to debounce (w/ active high v low)
    *Parameterize DEBOUNCE, and HOLD values
    *Make a pushbutton structure, obviating the need for global state vars

If we're making libraries, lets do something about RTCC.  It's begging
    for a simple API interface.

Start properly using the RTCC lock/unlock capability to protect RTCC from
    unintentional modifications.

PIC18F24J11 driven LED binary wrist watch.

