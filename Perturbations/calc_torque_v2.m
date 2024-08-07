% This function calculates ray-traces the wind on each point in the
% satellites and calculates a total torque for the given attitude. Variable
% "volume" here is the rotated point cloud so that the satellitely is 
% wind facing.
%
% -----------------------------------------------------------------------------
%   Copyright (c) 2010-2018 Samir A. Rawashdeh
%   Electrical and Computer Engineering
%   University of Michigan - Dearborn
%  
%   All rights reserved. 
%   
%   Redistribution and use in source and binary forms, with or without 
%   modification, are permitted provided that the following conditions are 
%   met:
%   
%       * Redistributions of source code must retain the above copyright 
%         notice, this list of conditions and the following disclaimer.
%       * Redistributions in binary form must reproduce the above copyright 
%         notice, this list of conditions and the following disclaimer in 
%         the documentation and/or other materials provided with the distribution
%         
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
%   POSSIBILITY OF SUCH DAMAGE.
%  
% ----------------------------------------------------------------------------
%

function [T, A_drag] = calc_torque_v2(volume, origin, Cd, resolution)

% from SMAD:
% T = 0.5 * Cd * rho * V^2 * A ( Uv x Scp )

%A_patch = 0.005*0.005; %This is the patch area 0.5cm x 0.5cm
A_patch = (resolution/100)*(resolution/100); %This is the patch area 0.25cm x 0.25cm
Uv = [-1 0 0];      %direction of the wind

T = [0 0 0];

for y = 1:size(volume,2)
    for z = 1:size(volume,3)
        w = find(volume(:,y,z) == 1);
        if ~isempty(w)
            T = T + 0.5 * Cd * A_patch * cross(Uv, (resolution/100)*([w(1),y,z]-origin));
        end
    end
end

A_drag = 0;
for y = 1:size(volume,2)
    for z = 1:size(volume,3)
        w = find(volume(:,y,z) == 1);
        if ~isempty(w)
            %A_drag = A_drag + A_patch * norm(cross(Uv, (resolution/100)*([w(1),y,z]-origin)));
            A_drag = A_drag + A_patch ;
        end
    end
end