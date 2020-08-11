
function [ resampled_particle_list ] = resampleSystematic( particle_list )
N = height(particle_list);
Q = cumsum(particle_list.weight);

T = ([1:N]-1 + rand(1))/N;

i=1;
j=1;

while (i<=N)
    if (T(i)<Q(j))
        resample_index(i)=j;
        i=i+1;
    else
        j=j+1;        
    end
    
    if j > N
        error('j is larger than N')
    end
end

resampled_particle_list = particle_list(resample_index,:);

end
