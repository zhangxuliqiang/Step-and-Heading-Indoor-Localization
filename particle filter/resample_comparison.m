N = height(particle_list);

u=([0:N-1]'+(rand(N,1)))/N;
wc=cumsum(particle_list.weight) ; 
wc=wc/wc(N) ; 
[dum,ind1]=sort([u;wc]); 
ind2=find(ind1<=N); 
resample_index = ind2-(0:N-1)'; 


resample_index2 = [];
% N = height(particle_list);
Q = cumsum(particle_list.weight);

T = u;

i=1;
j=1;

while (i<=N)
    if (T(i)<Q(j))
        resample_index2(i)=j;
        i=i+1;
    else
        j=j+1;
    end

    if j > N
        error('j is larger than N')
    end
end

resample_index2' == resample_index