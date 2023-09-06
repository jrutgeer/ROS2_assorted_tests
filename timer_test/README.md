
This is a small test application to check ROS 2 timer behavior in case of a timer callback that (always or occasionally) takes longer than the timer period.

Try changing:
- The type of callback group (reentrant vs mutually exclusive),
- The sleep duration (i.e. how long the timer callback takes),
- How often the timer callback sleeps (e.g always vs only one in five callbacks),
- The nr of threads for the multithreaded executor.
to see how this affects the callback calling behavior.

</br>
</br>

My conclusions:

Consider a timer at a rate of 1s,
- The first callback call however takes 3s calculation time,
- The next calls take only a few milliseconds.

</br>
</br>

In case of a **mutually exclusive** callback group:

Then:
- The first call will be made at t = 1s,
- This blocks execution of the 2nd call (obviously since mutually exclusive),
- The 2nd call will start instantly after completion of the 1st call (i.e. at t = 4s),
- The 3rd call will start at t = 5s,
- The 4th call will start at t = 6s,
- Etc.

Initial conclusion (not entirely correct):
I.e. desired callback time is defined as: previous callback start time + timer period.
Otherwise calls 2, 3 and 4 would all be made asap after completion of call 1 and call 5 at t = 5s.
Correct conclusion:
See [here](https://discourse.ros.org/t/ros-2-timer-behavior-when-callbacks-take-longer-than-the-timer-period/32536/2): timer aligns itself to the initial 'phase' of the timer.

E.g. consider timer with period 1s, and each call takes 0.001s except the 5th call takes 3,5s and the 6th call takes 2,2s.
This yields following times:

```
         Starts at time     Finishes at time
Call 1:          1                1.001
Call 2:          2                2.001
Call 3:          3                3.001
Call 4:          4                4.001
Call 5:          5                8.5
Call 6:          8.5              10.7
Call 7:         10.7              10.701
Call 8:         11                11.001
Call 9:         12                12.001
```

</br>
</br>

In case of a **reentrant** callback group:

- The first call is made at t = 1s, runs until t = 4s,
- The 2nd call is started at t = 2s in another (available) thread,
- The 3rd call is started at t = 3s also in an available thread,
- Etc.

As long as there are enough available threads (and computing power), then it does not matter if the callback takes longer than the timer period, the callbacks will still be called at the correct rate.
However if the executor runs out of available threads, then similar behavior as in the mutually exclusive case is obtained (i.e. execution of the next call is blocked until a previous one is done).


