function time = stamp2time(stamp)
    %{
    Inputs:
    stamp - ROS stamp
    start - start time in seconds
    %}
    s = stamp.Sec;
    ns = stamp.Nsec;
    time = double(s) + double(ns)/1E9;
end