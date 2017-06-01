
filename = 'Path1.txt';

clf('reset')
delimiterIn = ',';
headerlinesIn = 0;
A = importdata(filename,delimiterIn,headerlinesIn);

tpr = 410;
tpi = 102
target = A(1,7);
maxvel = 16
speed = 1
currentpos = 0;
currentvel = 0;
postolerance = 10;
veltolerance = 10;
error = 0;
preverror = 0;
P =0.045;
I = 0.000095;
D =  0.001;
feedback = 0;
flog = [];
timedelta = 0.1;
timecount = 1;
integral = 0;
integralthreshold = target * 0.25;
integralthreshold2 = target * 0.25;
integral2 = 0;
graph = [];
lastpos = 0;
plog = [];
pterm = 0;
iterm = 0;
dterm = 0;
dlog = [];
lastfeedback = 0;
ppos = 0;
dpos = 0;
perror = 0;
derror = 0;
ponly = 0;
donly = 0;
pprev = 0;
dprev = 0;
ppp = 0;
ppd = 0;
mlog = [];
target2 = A(1,8);
error2 =0;
preverror2=0;
lastpos2=0;
dterm2=0;
iterm2=0;
pterm2=0;
currentpos2 = 0;
derivative2 = 0;
feedback2 = 0;
figure('units','normalized','outerposition',[0 0 1 1])
load('pqfile.mat','maxfeedback','maxfeedback2');
%maxfeedback = 0;
for(k = 1:1:length(A))
    target = A(k,7);
    target2 = A(k,8);
   preverror = error;
   preverror2 = error2;
   pprev = perror;

   lastpos = currentpos;
   lastpos2 = currentpos2;
   ppp = ppos;

    error = target - currentpos;
    error2 = target2 - currentpos2;
     perror = target - ppos;
   
    
    
    
    derivative = (error - preverror)/ timedelta ; 
    derivative2 = (error2 - preverror2) / timedelta;
    pterm = error * P;
    pterm2 = error2 * P;
    ponly = perror * P;
    dterm = derivative * D;
    dterm2 = derivative2 * D;
    
    if(abs(error) < integralthreshold) 
		integral = integral + error;
	
	else 
		integral = 0;
    end
    
    if(abs(error2) < integralthreshold2) 
		integral2= integral2 + error2;
	
	else 
		integral2 = 0;
    end
    
    
    feedback = P * error + dterm + integral * I;
    feedback2 = P * error2 + dterm + integral2 * I;
    
    graph(1,timecount) = feedback2; %/ maxfeedback;
    graph(3,timecount) = feedback; %/ maxfeedback;
    graph(2,timecount) = timecount;
    graph(4,timecount) = dpos;
    
    
     flog(timecount) = feedback;
     plog(timecount) = feedback2;
     dlog(timecount) = feedback2/maxfeedback;
     mlog(timecount) = feedback / maxfeedback;
     if(length(flog) > 3)
     currentpos = currentpos + flog(k-2) * maxvel * timedelta;
     currentpos2 = currentpos2 + plog(k-2) * maxvel * timedelta
     else
         currentpos = currentpos;
         currentpos2 = currentpos2;
         ppos = ppos;
         dpos = dpos;
     end
     
     
    
    if(feedback > maxfeedback)
        maxfeedback = feedback;
    end
    if(feedback2 > maxfeedback)
        maxfeedback = feedback2;
    end
    
    
    timecount = timecount + 1;
end

%plot(graph(2,6:75),graph(1,6:75));
%plot(graph(2,6:75),graph(4,6:75));
%plot(graph(2,6:length(A.data) / 2 - 1),graph(3,6:length(A.data) / 2 - 1));
save('pqfile.mat','maxfeedback','maxfeedback2')
    
    
 g =  subplot(2,2,[1,3])  
i = imread('field.jpg');
l = imagesc([0 144], [0 144], i);
axis([0, 144, 0 ,144]);

%x = [];
%    y = []

h = animatedline('Color','r','LineWidth',3);
h1 = animatedline('Color','r','LineWidth',3);
xlabel('Animated Position view');
subplot(2,2,2)
xlabel('PID Simulated Motor Powers by time, left side')
h2 = animatedline('Color','b','LineWidth',1);
subplot(2,2,4)
xlabel('PID Simulated Motor Powers by time,right side');
h3 = animatedline('Color','k','LineWidth',1);
for k = 1:speed :length(A) -5
   
    addpoints(h,A(k:k+speed,3),A(k:k+speed,4));
   
    addpoints(h1,A(k:k+speed,5),A(k:k+speed,6));
   drawnow
   
    addpoints(h2,graph(2,k:k+speed),graph(1,k:k+speed));
    addpoints(h3,graph(2,k:k+speed),graph(3,k:k+speed));
    drawnow
end
    %}


%}