function data = add_timeinfo(data, time)
num_data = size(data, 1)-1;
start_t = time(1);
final_t = time(2);
timeline = [start_t:(final_t-start_t)/num_data:final_t]';
data = [data timeline];