function qlearn

% 0) Create map, Draw the map
%
% It should show Q values, Destination.  
% For each episode, update the Q values.
%

small_map = ...
      [ C, C, C, C;
        C, O, O, C;
        C, S, O, C;
        C, O, O, G;
        ];
    
map = small_map;

% 1) convert map to immediate reward matrix (node base)
% 100  - goal
% 1    - clear path
% -inf - obstacle



% 2) set up constants alpha, gamma, probabilities of movement, 
% num of episodes

% 3) loop thru num of episodes

% 4) Traverse from any starting point