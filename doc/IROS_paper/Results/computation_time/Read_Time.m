function Data=Read_Time(file_name)

FID=fopen(file_name);
fgetl(FID);
fgetl(FID);

kD=0;
while(1)
    slices=my_split(fgetl(FID)); 
    if(strcmp('</Times>',slices{1}) == 1)
        break;
    end    
    
    kD=kD+1;
    Data{kD}=Read_Tag(FID);
end

fclose(FID);

end

function slices=my_split(line)

slices=strsplit(strtrim(line));

end

function V=slices_2_vec(slices)

V=zeros(1,length(slices));
for k=1:length(slices)
    V(1,k)=str2num(slices{k});
end

end

function Data=Read_Tag(FID)

kr=0;
while(1)
    slices=my_split(fgetl(FID)); 
    
    if(slices{1}(1) == '<')
        break;
    end
    
    kr=kr+1;
Data(kr,:) = slices_2_vec(slices);    
end

end