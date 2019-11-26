
close all; clear; clc;
clear hash_map;

%% Create map
map = HashMap();
[key1, val1] = map.get();
assert(isempty(key1));
assert(isempty(val1));
clear key1 val1;

%% Set keys
m = 1e4;
n = 5;
nv = 6;
key = rand(m, n);
val = rand(m, nv);
t = tic();
map.set(key, val);
fprintf('Setting %i key-value pairs: %.3f s.\n', m, toc(t));
t = tic();
[val1] = map.get(key);
fprintf('Getting %i key-value pairs: %.3f s.\n', m, toc(t));
assert(all(val(:) == val1(:)));
assert(all(map.is(key)));
assert(~any(map.is(key+10)));
clear val1;

%% Clear map
map.clear();
[key1, val1] = map.get();
assert(isempty(key1));
assert(isempty(val1));
clear key1 val1;
