function qlearn(input_map)
clc; clear all; close all;

%------------------------------------
% 1) Create map, Draw the map
%
% It should show Q values, Destination.
% For each episode, update the Q values.
%------------------------------------
global EPISODES; EPISODES = 200;
display(sprintf('INFO: max episode %d',EPISODES));
global STEPS; STEPS = 200;
display(sprintf('INFO: max step %d',STEPS));
global ALPHA; ALPHA = [0.80]; % learning rate
display(sprintf('INFO: alpha %0.2f',ALPHA));
global GAMMA; GAMMA = [0.75]; % discount rate
display(sprintf('INFO: gamma %0.2f',GAMMA));
global DETERM_WORLD; DETERM_WORLD = 0;
display(sprintf('INFO: deterministic world %d',DETERM_WORLD));
global EPSIL_GREEDY_PR; EPSIL_GREEDY_PR = 0.2;
display(sprintf('INFO: e-greedy prob %0.2f',EPSIL_GREEDY_PR));

global DESIRED_PR;     DESIRED_PR     = 0.6;
display(sprintf('INFO: desired prob %0.2f',DESIRED_PR));
global OTHER_DIR_PR;   OTHER_DIR_PR   = 0.3;
display(sprintf('INFO: other dir prob %0.2f',OTHER_DIR_PR));
global INPLACE_PR;     INPLACE_PR     = 0.1;
display(sprintf('INFO: remain inplace prob %0.2f',INPLACE_PR));

global S; S = 7;
global G; G = 8;
global C; C = 1;
global O; O = 0;

if( nargin == 0 )

small_map = ...
      [ S, C, C, C;
        C, C, O, C;
        C, O, O, C;
        C, C, C, G;
        ];
small_map2 = ...
      [ S, O, C;
        O, O, C;
        C, G, C;
        ];

small_map3 = ...
      [ S, C, C;
        O, C, C;
        C, C, G;
        ];

large_map = ...
      [ C, C, C, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, S, O, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        O, O, O, C, O, O, O, O, O, C;
        C, C, C, C, C, C, C, C, C, C;
        G, C, C, C, C, C, C, C, C, C;
        ];
      
no_path_large_map = ...
      [ C, C, C, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, S, O, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        O, O, O, O, O, O, O, O, O, O;
        C, C, C, C, C, C, C, C, C, C;
        G, C, C, C, C, C, C, C, C, C;
        ];
%map = small_map;
%map = large_map;
map = large_map;
elseif ( nargin == 1)
  map = input_map;
end



global ROW;
global COL;
[ROW,COL] = size(map);
display(sprintf('INFO: Map size = %d x %d',ROW,COL));
display(sprintf('INFO: 0 - obstacle | 1 - clear path | 7 - start | 8 - goal'));

%------------------------------------
% 2) Validate map, and capture start, goal position
% Draw the map if valid
%------------------------------------
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
  display('ERROR: Validate map: Too many start or goal');
  return;
end

% draw a map
axis([1 COL+1 1 ROW+1]);
grid on;
hold on;
set(gca,'XTick',[1:1:COL]);
set(gca,'YTick',[1:1:ROW]);
set(gca,'xaxislocation','top','ydir','reverse');

% plot start, goal, obstacles
plot(start_c+0.5,start_r+0.5,'ro');
plot(goal_c+0.5,goal_r+0.5,'ko');
for ri = 1:ROW
  for ci = 1:COL
    if(map(ri,ci)==O) % if it is a obstacle draw it
      plot(ci+0.5,ri+0.5,'kx');
    end
  end
end

%------------------------------------
% 3) convert map into immediate reward matrix (node base) using following
%
%     100  - goal
%     0    - clear path
%     -inf - obstacle, self, not reachable, or diagonal...
%------------------------------------
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

R;

%------------------------------------
% 4) loop thru num of EPISODES
%------------------------------------
for param_i = 1:size(ALPHA,2)

textbox(ROW,COL) = struct('up',[],'down',[],'left',[],'right',[]);
episode_tb_hldr = [];
steps_tb_hldr = [];

Q = zeros(size(R));
for e = 1:EPISODES
  steps = 1; % reset the number of steps
  s = randperm(num_states,1); % current state
  if(isempty(episode_tb_hldr))
    episode_tb_hldr = text(0.5,0.8,sprintf('episode:%d',e));
  else
    set(episode_tb_hldr,'String',sprintf('episode:%d',e));
  end
  while ( s ~= goal_state && steps <= STEPS )
    
    if(isempty(steps_tb_hldr))
      steps_tb_hldr = text(0.5,1,sprintf('step:%d',steps));
    else
      set(steps_tb_hldr,'String',sprintf('step:%d',steps));
    end
    
    % policy for selecting action goes here
    % TODO: add different policy routine: greedy, e-greedy, softmax
    avail_actions = find(R(s,:) >= 0); % find num of possible actions or exploration bonus
    num_actions = size(avail_actions,2);
    if ( num_actions > 0 )
    else
      break;  % started on an obstacle state
    end
    
    % get maximum q value for all states
    q_max = max(Q,[],2);
    
    if( DETERM_WORLD == 0 )
      tmp = rand();
      if( tmp >= EPSIL_GREEDY_PR )
        % random action
        action_taken = ( round(rand() * (num_actions - 1)) ) + 1;
      else
        % greedy action
        [tmp_state,action_taken] = max(avail_actions);
      end
      
      % transition function for observed state based on selected action
      ds = avail_actions(action_taken);
      os = transition_function(s,ds);
      %Q(s,ds) = Q(s,os) + ALPHA(param_i) * ( R(s,ds) + GAMMA(param_i) * q_max(ds) - Q(s,ds) );
      Q(s,ds) = Q(s,ds) + ALPHA(param_i) * ( R(s,ds) + GAMMA(param_i) * q_max(os) - Q(s,ds) );
      textbox = plot_value(Q,s,ds,textbox);
      s = os;
    else
      tmp = rand();
      if( tmp >= EPSIL_GREEDY_PR )
        % random action
        action_taken = ( round(rand() * (num_actions - 1)) ) + 1;
      else
        % greedy action
        [tmp_state,action_taken] = max(avail_actions);
      end
      
      fs = avail_actions(action_taken);
      Q(s,fs) = Q(s,fs) + ALPHA(param_i) * ( R(s,fs) + GAMMA(param_i) * q_max(fs) - Q(s,fs) );
      textbox = plot_value(Q,s,fs,textbox);
      s = fs;
    end

    steps = steps + 1;
  end
end


saveas(gcf,sprintf('a%0.2fg%0.2fe%d.jpg',ALPHA(param_i),GAMMA(param_i),EPISODES),'jpg');
end

Q;

%------------------------------------
% 5) Traverse from any starting point
%------------------------------------
qi = start_state;
plot_cnt = 0;
while qi ~= goal_state
  [max_value,indx] = max(Q(qi,:));
  if(max_value == 0)
    display('Did not converge');
    break;
  end
  [r,c] = indx2rc(qi);
  [fr,fc] = indx2rc(indx);
  if( c > fc )
    xx = [fc+0.5 c+0.5];
  else
    xx = [c+0.5 fc+0.5];
  end
  
  if( r > fr )
    yy = [fr+0.5 r+0.5];
  else
    yy = [r+0.5 fr+0.5];
  end
  plot(xx,yy);
  qi = indx;
  plot_cnt = plot_cnt + 1;
  if( plot_cnt > num_states )
    display('Did not converge');
    break;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%--------------------------------------------------------------------------
% transition function
%--------------------------------------------------------------------------
function observed_state = transition_function(s,a)
global DESIRED_PR;
global OTHER_DIR_PR;
global INPLACE_PR;
global ROW;
global COL;


bucket(1) = DESIRED_PR;
bucket(2) = bucket(1) + OTHER_DIR_PR;
bucket(3) = bucket(2) + INPLACE_PR;

r_action = rand();

if( r_action <= bucket(1) )
  observed_state = a;
elseif( r_action <= bucket(2) )
  RIGHT = 1; DOWN = 2; LEFT = 3; UP = 4;
  [r,c] = indx2rc(s);
  [fr,fc] = indx2rc(a);
  avail_actions = [];
  if ( fr - r == -1 )      % UP
    avail_actions = [RIGHT DOWN LEFT   ];
  elseif ( fc - c == 1)    % RIGHT
    avail_actions = [      DOWN LEFT UP];
  elseif ( fr - r == 1 )   % DOWN
    avail_actions = [RIGHT      LEFT UP];
  elseif ( fc - c == -1 )  % LEFT
    avail_actions = [RIGHT DOWN      UP];
  else
    display('bug 1');
  end
  
  rand_var = rand();
  if(rand_var <= 0.33)
    indx = 1;
  elseif(rand_var <= 0.66)
    indx = 2;
  elseif(rand_var <= 1)
    indx = 3;
  else
    display('bug 2');
  end
  
  if( avail_actions(indx) == RIGHT )
    if( c + 1 <= COL )
      fc = c + 1;
      observed_state = rc2indx(fr,fc);
    else
      observed_state = s;
    end
  elseif( avail_actions(indx) == DOWN )
    if( r + 1 <= ROW )
      fr = r + 1;
      observed_state = rc2indx(fr,fc);
    else
      observed_state = s;
    end
  elseif( avail_actions(indx) == LEFT )
    if( c - 1 >= 1 )
      fc = c - 1;
      observed_state = rc2indx(fr,fc);
    else
      observed_state = s;
    end
  elseif( avail_actions(indx) == UP )
    if( r - 1 >= 1 )
      fr = r - 1;
      observed_state = rc2indx(fr,fc);
    else
      observed_state = s;
    end
  else
    display('bug 3');
  end
  
  %display('not picking orig action');
else
  observed_state = s;
end

%--------------------------------------------------------------------------
% textbox
% Draws Q values on map
%--------------------------------------------------------------------------
function textbox = plot_value(Q,s,fs,textbox)
% update plot
[r,c] = indx2rc(s);
[fr,fc] = indx2rc(fs);

str = sprintf('%0.2f', Q(s,fs));
if ( fr - r == 1 )  % r+1
  if(isempty(textbox(r,c).down))
    textbox(r,c).down = text(c+0.5, r+0.9,str);
  else
    set(textbox(r,c).down,'String',str);
  end
elseif ( r - fr == 1) % r-1
  if(isempty(textbox(r,c).up))
    textbox(r,c).up = text(c+0.5, r+0.1,str);
  else
    set(textbox(r,c).up,'String',str);
  end
elseif( fc - c == 1) % c+1
  if(isempty(textbox(r,c).right))
    textbox(r,c).right = text(c+0.8, r+0.5,str);
  else
    set(textbox(r,c).right,'String',str);
  end
elseif( c - fc == 1) % c-1
  if(isempty(textbox(r,c).left))
    textbox(r,c).left = text(c+0.05, r+0.5,str);
  else
    set(textbox(r,c).left,'String',str);
  end
else
  display('should not be here, bug');
end
%pause(0.001);
drawnow;

%--------------------------------------------------------------------------
% rc2indx
%--------------------------------------------------------------------------
function index = rc2indx(r,c)
global ROW;
global COL;
index = (r-1)*COL+c;

%--------------------------------------------------------------------------
% indx2rc
%--------------------------------------------------------------------------
function [r,c] = indx2rc(i)
global ROW;
global COL;
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end
