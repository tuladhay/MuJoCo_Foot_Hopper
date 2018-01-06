% Plot the results for foot hopper RL

filename = "progress.csv";
data = csvread(filename,1,0);
avg_dis_return = data(:,7);
avg_return = data(:,16);

figure(1)
plot(avg_return);
title("Average return");

figure(2)
plot(avg_dis_return);
title("Average discounted return");
