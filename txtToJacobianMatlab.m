function []=txtToJacobianMatlab(file,JacSize)
n=strsplit(file,'.');
name=n{1};
%% Read File
f=fopen(file);
rawT=textscan(f,'%s','Delimiter','=')
text=rawT{1}(1:2:end)
t=strsplit(text{1},'[')

for i=1:size(text)
    t=strsplit(text{i},'[');
index1=str2num(t{2}(1:end-1));
index2=str2num(t{3}(1:end-2));
tf(i)=strcat(t{1},'(',num2str(index1+1),',',num2str(index2+1),')','= ',rawT{1}(i*2));
end
fclose(f)

%% Write File
fname=strcat(name,'.m');
w=fopen(fname,'w');
firstLine=strcat('function ',name,' = get',name,' (q)');
for i=1:JacSize(2)
secondLine=strat(secondLine,'q',num2str(i),' = q(',num2str(i),'); ');
end
thirdLine=strat(name,' = zeros(',num2str(JacSize(1)),',',num2str(JacSize(2)),');');

end