function []=ParticleFilter(fwd_Noise,turn_Noise,sense_Noise,turn_Angle,distance,grid_Size,no_Particles,no_Robots,no_Iterations)
myRobot=robot(grid_Size);
C=[1,0,0;0,1,0;0,0,1];
N=no_Particles;

for i=1:N
    particle=robot(grid_Size);
    particle=particle.SetNoise(fwd_Noise,turn_Noise,sense_Noise);
    P(i)=particle;
    clear x;
end

for i=1:N
    X(i)=P(i).x;
    Y(i)=P(i).y;
    O(i)=P(i).orientation;
end

disp(myRobot.eval(P));
for k=1:no_Iterations
    
    scatter(X,Y,10,[C(1,:)]);
    set(gca,'XLim',[0,grid_Size],'YLim',[0,grid_Size]);
    drawnow;
%     linkdata on;
    
    myRobot = myRobot.Move(turn_Angle,distance);
    Z = myRobot.Sense();
    
    for i=1:N
        P(i)=P(i).Move(turn_Angle,distance);
    end

    for i=1:N
        W(i)=P(i).measurementProb(Z);
    end

    norm=sum(W);
    W=W./norm; %normalized weight array

    mW=max(W);
    beta=0;
    index=randi(N);
    for i=1:N
        beta=beta+rand(1)*2*mW;
        while(beta>W(index))
            beta=beta-W(index);
            index=index+1;
            if(index==N)
                index=index-N+1;
            end
        end
        P1(i)=P(index);
    end
    P=P1;
    
    for i=1:N
    X(i)=P(i).x;
    Y(i)=P(i).y;
    O(i)=P(i).orientation;
    end
    
    disp(myRobot.eval(P));
end

for i=1:N
    X(i)=P(i).x;
    Y(i)=P(i).y;
    O(i)=P(i).orientation;
end
end
