xs = 1:100;

ys = 1:100;


for i = 1:100
    ys(i) =  i*i*i*i;
end

plot(xs,ys)