README for CTP_event-based sensor
Author/Contact: behdad@kth.se

Description:

The event-based sensor send data whenever the event condition is true which decides if the sensor node transmits the most recent tank levels is given by:
|L2(t) − L2(ts)| > elim OR hact  hlim,
where L2(ts) is the last transmitted lower tank level, elim the
threshold value, hact the time elapsed since the last transmission
and hlim is the maximum allowed inter-transmission time.
The thresholds are set to elim = 0.2 and hlim = 10 s.


Tools:

Known bugs/limitations:

None.

