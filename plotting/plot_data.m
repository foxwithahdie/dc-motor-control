function plot_data()
    % Read data
    data = readtable("../data/speed_and_sensor_data.csv");
    time = linspace(0, 30, length(data.left_speed_data));
    figure();
    
    yyaxis left
    plot(time, data.left_speed_data, "-", Color="blue", LineWidth=3, DisplayName="Left Speed"); hold on
        plot(time, data.right_speed_data, "-", Color="red", LineWidth=3, DisplayName="Right Speed");
    yyaxis right
        plot(time, data.left_sensor_data, "--", Color=[0, 0.5, 1], LineWidth=1, DisplayName="Left Sensor");
        plot(time, data.right_sensor_data, "--", Color=[1, 0.5, 0], LineWidth=1, DisplayName="Right Sensor");
        legend();
        title("Sensor Data and Speed over Time");
        xlabel("Time (s)");
    hold off
end