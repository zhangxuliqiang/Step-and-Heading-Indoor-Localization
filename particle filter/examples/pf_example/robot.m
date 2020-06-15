classdef robot
    
    properties(Constant=true)
        LandMarks  = [[20.0, 20.0]; [80.0, 80.0]; [20.0, 80.0];[80.0, 20.0]];
        
    end
    
    properties
        x=0;
        y=0;
        orientation=0;
        forwardNoise=0;
        turnNoise=0;
        senseNoise=0;
        worldSize=100;
    end
  
    methods
        function obj=robot(worldSize)
%             if(nargin>0)
                obj.worldSize=worldSize;
                obj.x=rand()*worldSize;
                obj.y=rand()*worldSize;
                obj.orientation=rand()*2*pi;
                obj.forwardNoise=0;
                obj.turnNoise=0;
                obj.senseNoise=0;
                
%             end
        end
        function obj=Set(obj,newX,newY,newOrientation)
            if(newX<0 || newX>obj.worldSize)
                display('X Coordinate out of bound');
            end
            if(newY<0 || newY>obj.worldSize)
                display('Y Coordinate out of bound');
            end
            if(newOrientation<0 || newOrientation >2*pi)
                display('Orientation must be in [0-2*Pi]');
            end
            obj.x=newX;
            obj.y=newY;
            obj.orientation=newOrientation;
        end
        
        function obj=SetNoise(obj,newFnoise,newTnoise,newSnoise)
            obj.forwardNoise=newFnoise;
            obj.turnNoise=newTnoise;
            obj.senseNoise=newSnoise;
        end
        
        function Z=Sense(obj)
            Z=[];
            [m,n]=size(obj.LandMarks);
            for i=1:m
                dist=sqrt((obj.x-obj.LandMarks(i,1))^2+(obj.y-obj.LandMarks(i,2))^2);
                dist=dist+normrnd(0,obj.senseNoise);
                Z=[Z,dist];
            end
        end
        
        function obj= Move(obj,turn,forward)
            if(forward<0)
                display('Error, forward cannot be less than 0');
            end
            orientationLocal=obj.orientation+(turn)+normrnd(0,obj.forwardNoise);
            orientationLocal=mod(orientationLocal,2*pi);

            dist=(forward)+normrnd(0,obj.forwardNoise);
            X=obj.x+cos(orientationLocal)*dist;
            Y=obj.y+sin(orientationLocal)*dist;
            X=mod(X,obj.worldSize);
            Y=mod(Y,obj.worldSize);

%             myRobot=robot();
            obj=obj.Set(X,Y,orientationLocal);
            obj=obj.SetNoise(obj.forwardNoise,obj.turnNoise,obj.senseNoise);
%             obj=myRobot
        end
        
        function ValueG=Gaussian(obj,mu,sigma,x)
            k=0.5*(((x-mu)^2)/(sigma^2));
            ValueG= (exp(-k))/(sqrt(2*pi*sigma^2));
        end
        
        function prob=measurementProb(obj,measurement)
            prob=1;
            [m,n]=size(obj.LandMarks);
            for i=1:m
                dist=sqrt((obj.x - obj.LandMarks(i,1))^2 + (obj.y - obj.LandMarks(i,2))^2);
                prob=prob*obj.Gaussian(dist,obj.senseNoise,measurement(i));
            end
        end
        
%         function obj=Repair(obj)
%             return []
%         end
        
        function Result=eval(obj,p)
            sum=0;
            [m,n]=size(p);
            for i=1:n
                dx=mod((p(i).x-obj.x+(obj.worldSize/2)),obj.worldSize)-obj.worldSize/2;
                dy=mod((p(i).y-obj.y+(obj.worldSize/2)),obj.worldSize)-obj.worldSize/2;
                err=sqrt(dx*dx+dy*dy);
                sum=sum+err;
            end
            Result= sum/n; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
    
            
            
                
                
        
        
         