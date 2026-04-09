function est_RBP = usr_REGENBP(ACC,RBP)

coeff = exp(0.0411./abs(ACC)).^-1;
est_RBP = 0.125*coeff.*RBP;

end