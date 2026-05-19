K=1.1 #the factor by which B is increased
M=3 # number of facilities
D=3 #Id has D*D dimension
d=[5.1,9.2,2.3,1.1] #list of cost incurred has M+1 elements from k=0 to k=M
#dk is the cost incurred in travelling from gamma k to gammak+1
import numpy
import math

Id = numpy.eye(D)
print(Id)

def p(k,B):
    #For numerator
    numerator=0
    for i in range(k+1,M+1):
        sum1=0
        for t in range(i,M+1):
            sum1+=d[t]
        numerator+=(math.e)**(-B*sum1)
        
    #For denomenator
    denomenator=0
    for i in range(k,M+1):
        sum1=0
        
        for t in range(i,M+1):
            sum1+=d[t]
        denomenator+=(math.e)**(-B*sum1)
        
# %%   
    output=((math.e)**(-B*d[k]))*numerator/denomenator
    return output
def main(X,z,B_min,B_max):
    
    B=B_min
    while B<B_max:
        B=K*B