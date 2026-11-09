function result = analyze_imu_calibration(path)
% Protocol v1: 36-byte little-endian packets; final sample has flags bit 0.
if nargin==0
 [f,p]=uigetfile({'*.bin;*.log','RTT binary log'}); if isequal(f,0), result=[];return;end
 path=fullfile(p,f);
end
fid=fopen(path,'rb','ieee-le');assert(fid>=0,'Cannot open log');cleanup=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); means=[]; current=[]; previous=0; times=[]; windows=[];
k=1;
while k+35<=numel(b)
 if ~isequal(b(k:k+3)',uint8([73 85 77 49])) || b(k+4)~=1, k=k+1;continue;end
 p=b(k:k+35);k=k+36;
 seq=double(typecast(p(9:12),'uint32'));t=double(typecast(p(13:20),'uint64'));
 a=double(typecast(p(21:26),'int16'))';idx=double(typecast(p(35:36),'uint16'));
 if idx==1,current=[];times=[];end
 if idx~=size(current,1)+1 || (~isempty(current) && (seq~=previous+1 || t<=times(end)))
  current=[];times=[];continue;
 end
 current(end+1,:)=a;times(end+1)=t;previous=seq;
 if bitand(p(8),1) && idx==2048
  if ~ismember(p(7),windows),means(end+1,:)=mean(current,1);windows(end+1)=p(7);end
  current=[];times=[];
 end
end
assert(size(means,1)>=16,'Need at least 16 complete windows');
x=means*(24/32768); % normalize optimizer variables to g
test=1:4:size(x,1);train=setdiff(1:size(x,1),test);
d=fit(x(train,:),false);full=fit(x(train,:),true);
rd=rms(vecnorm((x(test,:)-d.bias)*d.matrix',2,2)-1);
rf=rms(vecnorm((x(test,:)-full.bias)*full.matrix',2,2)-1);
selected=d;if cond(full.matrix)<100 && rf<0.9*rd,selected=full;end
result=struct('means',means,'bias',selected.bias/(24/32768),'matrix',selected.matrix*(24/32768)*9.8048,'diagonalTestRms',rd,'fullTestRms',rf);
fprintf('Diagonal/full validation RMS (g): %.6g / %.6g\n',rd,rf);
fprintf('inline constexpr std::array<float,3> accel_bias_raw{{');fprintf('%.9eF,',result.bias);fprintf('}};\n');
fprintf('inline constexpr std::array<float,9> accel_correction_matrix{{');fprintf('%.9eF,',reshape(result.matrix',1,[]));fprintf('}};\n');
end
function model=fit(x,full)
p=zeros(1,6+3*full);
objective=@(p)sum((vecnorm((x-p(1:3))*matrix(p)',2,2)-1).^2);
p=fminsearch(objective,p,optimset('Display','off','MaxIter',20000,'MaxFunEvals',40000,'TolX',1e-10,'TolFun',1e-12));
model=struct('bias',p(1:3),'matrix',matrix(p));
end
function m=matrix(p)
m=diag(exp(p(4:6)));if numel(p)==9,m(1,2)=p(7);m(1,3)=p(8);m(2,3)=p(9);end
end

