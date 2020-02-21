function xlpf = LPF(x)
% 1�� Low Pass Filter
% 2011/07/13
persistent prevX;
persistent firstRun;
      
if isempty(firstRun)
    prevX = x;
    
    firstRun = 1;
end

alpha = 0.95;
xlpf = alpha*prevX+(1-alpha)*x;

prevX = xlpf;