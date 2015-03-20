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
        C, O, C, G;
        ];
small_map2 = ...
      [ S, O;
        C, G;
        ];

map = small_map;


global ROW;
global COL;
[ROW,COL] = size(map);

% Validate map, and capture start, goal position
num_start = 0;
num_goal = 0;
for r = 1:ROW
  for c = 1:COL
    if( map(r,c) == S )
      num_start = num_start + 1;
      start_r = r; start_c = c;
    end
    if( map(r,c) == G )
      num_goal = num_goal + 1;
      goal_r = r; goal_c = c;
    end
  end
end
if( num_start > 1 || num_goal > 1 )
  display('Too many start or goal');
  exit 0;
end

% 1) convert map to immediate reward matrix (node base)
% 100  - goal
% 0    - clear path
% -inf - obstacle, self, not reachable, diagonal...

num_states = ROW*COL;
start_state = rc2indx(start_r,start_c);
goal_state = rc2indx(goal_r,goal_c);
R(num_states,num_states) = 0;

% 1st loop represent current node
% 2nd loop represent visiting node, check if it is clear path, or obstacle
for qx = 1:num_states
    [cr,cc] = indx2rc(qx);  % (c)urrent (r)ow and (c)ol
    if( map(cr,cc) == O ) % current node is an obstacle
      R(qx,:) = -inf;
      continue;
    end
    for qy = 1:num_states
        [vr,vc] = indx2rc(qy);
        if ( abs(vr-cr) == 1 && cc == vc ) % +/1 row
          if ( map(vr,vc) == C || map(vr,vc) == S )
            R(qx,qy) = 0;
          elseif( map(vr,vc) == G )
            R(qx,qy) = 100;
          else
            R(qx,qy) = -inf;
          end
        elseif (abs(vc-cc) == 1 && cr == vr ) % +/- col
          if ( map(vr,vc) == C || map(vr,vc) == S )
            R(qx,qy) = 0;
          elseif( map(vr,vc) == G )
            R(qx,qy) = 100;
          else
            R(qx,qy) = -inf;
          end
        else % diagonal
          R(qx,qy) = -inf;
        end
    end
end


% 2) set up constants alpha, gamma, probabilities of movement, 
global ALPHA; ALPHA = 0.1;
global GAMMA; GAMMA = 0.2;
global EPISODES; EPISODES = 10;

Q = zeros(size(R));
max_while = 1;
% 3) loop thru num of EPISODES
for e = 1:EPISODES
  s = randperm(num_states,1); % current state
  cnt = 0;
  while s ~= goal_state
    actions = find(R(s,:) >= 0) % find num of possible actions
    num_actions = size(actions,2)
    if ( num_actions > 0 )
      % policy for selecting action goes here
      action_taken = ( round(rand() * (num_actions - 1)) ) + 1
    else
      display('not possible');
    end

    q_max = max(Q,[],2);
    Q(s,action_taken) = R(s,action_taken) + GAMMA * q_max(action_taken);
    s = action_taken;
    cnt = cnt + 1;
    if( cnt > max_while )
      break;
    end
  end
end

Q

% 4) Traverse from any starting point



%--------------------------------------------------------------------------
% util
%--------------------------------------------------------------------------
function index = rc2indx(r,c)
global ROW;
global COL;
index = (r-1)*COL+c;

function [r,c] = indx2rc(i)
global ROW;
global COL;
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end
