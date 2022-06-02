function plot_ekf_timing(ekf_msgs)
    % gnc/ekf timing data
    timing = [];
    for i = 1:1:(length(ekf_msgs)-1)
        t1 = stamp2time(ekf_msgs{i}.Header.Stamp);
        t2 = stamp2time(ekf_msgs{i+1}.Header.Stamp);
        timing(i) = t2 - t1;
    end
    plot(timing)
    mean(timing)
    yline(1.0/62.5)
    
    % time vs. idx
    figure;
    time = [];
    t0 = stamp2time(ekf_msgs{1}.Header.Stamp);
    tf = stamp2time(ekf_msgs{end}.Header.Stamp);
    for i = 1:1:length(ekf_msgs)
        time(i) = stamp2time(ekf_msgs{i}.Header.Stamp) - t0;
    end
    start_time = datetime(t0, 'convertfrom', 'posixtime', 'Format', 'MM/dd/yy HH:mm:ss.SSS')
    end_time = datetime(tf, 'convertfrom', 'posixtime', 'Format', 'MM/dd/yy HH:mm:ss.SSS')
    
    plot(1:length(ekf_msgs), time, '*');
end