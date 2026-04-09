function consumption = usr_MT2BC_sys(MT,SP,sys)
% x = [1 MT MT^2 MT^3];
x = [ones(size(MT,1),1) MT MT.^2 MT.^3];
y = [ones(size(MT,1),1) SP SP.^2 ones(size(MT,1),1)];

CalA = sys*x'; % 4 by x
 % consumption = sum(y.*fliplr(CalA'),2);

 consumption = sum(y.*(CalA'),2);

end