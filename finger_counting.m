% % Copyright (c) 2016, ZHIHAO ZHANG JIAHAO LI, University of Alberta.
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without modi-
% fication, are permitted provided that the following conditions are met:
%  1. Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%  2. Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% This code is created to count the number of the finger
% you can use use this in any purposes  
% Happy coding 
% YEAH!!!!
%%
function [] = finger_counting()
clear;
clc;
close all; % as ususal
videoobject=VideoReader('Movie_12345.mov'); % aquire frame from the video


while hasFrame(videoobject)
    img=readFrame(videoobject);
    birgb = filtered_binarilize_pic(img);
    birgb_clean = bwareaopen(birgb,200);
    %filtering out the less than 200 pixels area
    birgb_full = imdilate(birgb_clean,strel('diamond',4));
    cc=bwconncomp(birgb_full);
    %label the area within connectivity 8 and label them into 1 ,2 ,3 etc.
    arr=(cellfun('length',cc.PixelIdxList));
    
    if ~isempty(round(arr))
        msz=0;
        for i=1:length(arr)
            if msz<arr(i:i)
                msz=arr(i:i);
                index=i;
            end
        end
        % getting a biggest area and its index by using this loop
        labels=labelmatrix(cc);
        newLabel=(labels==index);
        out=newLabel;
    end
    out=imfill(out,'holes');
    stats=regionprops(out,'Centroid');
    if ~isempty(stats);
        final = palm_detection(stats,out);
        final=out-final;
        final=bwareaopen(final,200);
        final=imerode(final,strel('disk',10));
        % erode the edge of the palm because from out - final some edge 
        %could be remain on the edge of the palm
        final=bwareaopen(final,400);
        %filtering out the area less than 400
        final=imclearborder(final,8);
        % cleaning the border 
        a = bwareaopen(final,1800);
        % filtering out the area less than 2000 pixels
        [~,num]=bwlabeln(final,8);
        imshow(a);
        clc;
        fprintf('The detected number of fingers are %2d \n',num);
    end
end
end
%% function for binarilizing the image and filtering out colour
function [birgb] = filtered_binarilize_pic(Var1)
s_ize=size(Var1);
    r=1;g=2;b=3; % 1= red 2 = green 3 = blue
    rgb=Var1;
    r_gion=rgb;
    
    for i=1:s_ize(1) % column
        for j=1:s_ize(2) % row
            if rgb(i,j,g)>20 && rgb(i,j,g)<70 ; % getting from iamge threshold 
                r_gion(i,j,r)=255;
                r_gion(i,j,g)=255;
                r_gion(i,j,b)=255;% coverting pixels follow the requirement into white
                
            else
                r_gion(i,j,r)=0;
                r_gion(i,j,g)=0;
                r_gion(i,j,b)=0; % coverting pixels outside of requirement into black
            end
        end
    end
    
    birgb=im2bw(r_gion); %convert image into binary image

end

%%  plam detection
function [final] = palm_detection(Var1,Var2)
cx=Var1.Centroid(1);
cy=Var1.Centroid(2);
%find the nearest countour point
boundary=bwboundaries(Var2); % edge of the palm 
Dist=1080*720; % size of the video 
bImg=uint8(zeros(1080,720));  %prealocation 
for i=1:length(boundary) % num of points coordiante of boundary cell
    cell=boundary{i,1}; 
    for j=1:length(cell) % i th pixel of the boundary cell
        y=cell(j,1); 
        x=cell(j,2);
        sqrDist=(cx-x)*(cx-x)+(cy-y)*(cy-y); % distance from cnetrois to i th pixel of the edge
        bImg(x,y)=255;  %  illustate the edge of palm
        if(sqrDist<Dist) 
            Dist=sqrDist; % gving the lowest distance from centroid to edge 
        end
    end
end
shape=strel('disk',round(sqrt(Dist)/2)); % construct the circle in the radius of 'round(sqrt(Dist)/2)' 
final=imerode(Var2,shape); 
% erode the whole hand of the circle constucted by previous line 
% which can erode the finger automatically 
final=imdilate(final,shape);
% make up the eroded part of the palm expecting the fingers( finger is all removed) 

end
%%
