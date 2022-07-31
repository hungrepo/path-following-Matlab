mean_cp = [];
for i =1:7
    str1 = 'compute_time_';
    str2 = num2str(i);
    
filename = strcat(str1,str2);
load(filename)
mean_cp(i+1) = 1000*mean(compute_time);
end