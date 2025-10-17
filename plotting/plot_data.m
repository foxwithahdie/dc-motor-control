function plot_data()
    data = readtable("../data/speed_and_sensor_data.csv");
    time = linspace(0, 60, length(data.left_speed_data));
    figure();
    yyaxis left
    plot(time, data.left_speed_data, Color=[0, 0.8, 0.8], DisplayName="Left Speed"); hold on
        plot(time, data.right_speed_data, Color=[0.8, 0, 0.8], DisplayName="Right Speed");
    yyaxis right
        plot(time, data.left_sensor_data, Color=[0, 0.3, 0.3], DisplayName="Left Sensor");
        plot(time, data.right_sensor_data, Color=[0.8, 0.2, 0.2], DisplayName="Right Sensor");
        legend();
        title("Sensor Data and Speed over Time");
        xlabel("Time (s)");
    hold off

end