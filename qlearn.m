function qlearn

%clc; clear all; close all;

global S; S = 7;
global G; G = 8;
global C; C = 1;
global O; O = 0;

% 0) Create map, Draw the map
%
% It should show Q values, Destination.
% For each episode, update the Q values.

small_map = ...
      [ C, C, C, C;
        C, O, O, C;
        C, S, O, C;
        C, O, O, G;
        ];
small_map2 = ...
      [ S, O, C;
        C, G, C;
        ];

map = small_map2;

global ROW;
global COL;
[ROW,COL] = size(map);

% 1) convert map to immediate reward matrix (node base)
% 100  - goal
% 1    - clear path
% -inf - obstacle
% 0    - not reachable, or self

num_states = ROW*COL;
R(num_states,num_states) = 0;

% 1st loop represent current node
% 2nd loop represent visiting node, check if it is clear path, or obstacle
for qx = 1:num_states
    [cr,cc] = indx2rc(ROW,COL,qx);  % (c)urrent (r)ow and (c)ol
    if( map(cr,cc) == O )
      R(qx,:) = -inf;
      continue;
    end
    for qy = 1:num_states
        [vr,vc] = indx2rc(ROW,COL,qy);
        if ( abs(vr-cr) == 1 && cc == vc )
          if ( map(vr,vc) == C )
            R(qx,qy) = 1;
          elseif( map(vr,vc) == G )
            R(qx,qy) = 100;
          elseif( map(vr,vc) == S )
            R(qx,qy) = 1;
          else
            R(qx,qy) = -inf;
          end
        elseif (abs(vc-cc) == 1 && cr == vr )
          if ( map(vr,vc) == C )
            R(qx,qy) = 1;
          elseif( map(vr,vc) == G )
            R(qx,qy) = 100;
          elseif( map(vr,vc) == S )
            R(qx,qy) = 1;
          else
            R(qx,qy) = -inf;
          end
        else
          R(qx,qy) = 0;
        end
    end
end

R
% 2) set up constants alpha, gamma, probabilities of movement, 
% num of episodes

% 3) loop thru num of episodes

% 4) Traverse from any starting point



%--------------------------------------------------------------------------
% util
%--------------------------------------------------------------------------
function index = rc2indx(ROW,COL,r,c)
index = (r-1)*COL+c;

function [r,c] = indx2rc(ROW,COL,i)
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end
