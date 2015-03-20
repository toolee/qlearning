function qlearn

clc; clear all; close all;

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

map = small_map;

global ROW;
global COL;
[ROW,COL] = size(map);

% 1) convert map to immediate reward matrix (node base)
% 100  - goal
% 1    - clear path
% -inf - obstacle

num_states = ROW*COL;
Q(num_states,num_states) = 0;

% This is not working yet
% for r = 1:ROW
%     % initialize the whole row, just use 1 as col
%     i = rc2indx(ROW,COL,r,1);
%     Q(i,:) = -inf;
%     
%     for c = 1:COL
%         j = rc2indx(ROW,COL,r,c);
%         if( map(r,c) == C )
%             Q(i,j) = 1;
%         elseif( map(r,c) == G )
%             Q(i,j) = 100;
%         end
%         
%     end % for c = 1:COL
% end % for r = 1:ROW
% 
% for qi = 1:num_states
%     Q(qi,:) = -inf;
%     for qj = 1:num_states
%         [r,c] = indx2rc(ROW,COL,qj);
%         if( map(r,c) == C )
%             Q(i,j) = 1;
%         elseif( map(r,c) == G )
%             Q(i,j) = 100;
%         end
%     end
% end

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