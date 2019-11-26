classdef VoxelMap < handle
    %VOXELMAP 3-D voxel map

    properties
        map_id
        voxel_size = 1
        free_update = -1
        hit_update = 1
        occupied_threshold = 0
    end
    
    methods
        function obj = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold)
            %VOXELMAP
            %
            % obj = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold)
            obj.map_id = voxel_map('create_map');
            if nargin >= 1
                assert(isa(voxel_size, 'double') && isscalar(voxel_size) && voxel_size > 0);
                obj.voxel_size = voxel_size;
            end
            if nargin >= 2
                assert(isa(free_update, 'double') && isscalar(free_update));
                obj.free_update = free_update;
            end
            if nargin >= 3
                assert(isa(hit_update, 'double') && isscalar(hit_update));
                obj.hit_update = hit_update;
            end
            if nargin >= 4
                assert(isa(occupied_threshold, 'double') && isscalar(occupied_threshold));
                obj.occupied_threshold = occupied_threshold;
            end
        end
        function delete(obj)
            if ~isempty(obj.map_id)
                voxel_map('delete_map', obj.map_id);
                obj.map_id = [];
            end
        end
        
        function vox = voxels(obj, x)
            vox = floor(x ./ obj.voxel_size);
        end
        function x = voxel_centers(obj, vox)
            x = obj.voxel_size * (vox + 0.5);
        end
        
        function n = size(obj)
            %SIZE
            %
            % n = size(obj)
            n = voxel_map('size', obj.map_id);
        end
        
        function clear(obj)
            %CLEAR
            %
            % clear(obj)
            voxel_map('clear', obj.map_id);
        end
        
        function [varargout] = trace_rays(obj, from, dir, max_range, min_val, max_val)
            %TRACE_RAYS
            %
            % [x, val] = trace_rays(obj, from, dir, max_range, min_val, max_val)
            % [x, val, ray_x, ray_val] = trace_rays(...)
            if isempty(from) || isempty(dir)  % No rays to trace.
                [varargout{1:4}] = deal(zeros([3 0]), zeros([1 0]), {}, {});
                return;
            end
            assert(isa(from, 'double') && ismatrix(from) && size(from, 1) == 3);
            from = obj.voxels(from);
            assert(isa(dir, 'double') && ismatrix(dir) && size(dir, 1) == 3);
            [from, dir] = VoxelMap.same_cols(from, dir);
            if nargin < 4 || isempty(max_range)
                max_range = inf;
            end
            assert(isa(max_range, 'double') && isscalar(max_range) && max_range >= 0);
            max_range = max_range / obj.voxel_size;
            if nargin < 5 || isempty(min_val)
                min_val = -inf;
            end
            assert(isa(min_val, 'double') && isscalar(min_val));
            if nargin < 6 || isempty(max_val)
                max_val = obj.occupied_threshold;
            end
            assert(isa(max_val, 'double') && isscalar(max_val));
            
            [varargout{1:nargout}] = voxel_map('trace_rays', obj.map_id, ...
                from, dir, min_val, max_val, max_range);
            
            varargout{1} = obj.voxel_centers(varargout{1});
            if nargout >= 3
                varargout{3} = cellfun(@(x) obj.voxel_centers(x), varargout{3}, ...
                    'UniformOutput', false);
            end
        end
        function [x, val] = trace_lines(obj, from, to, min_val, max_val)
            %TRACE_LINES
            %
            % [x, val] = trace_lines(obj, from, to, min_val, max_val)
            if isempty(from) || isempty(to)
                x = zeros([3 0]);
                val = zeros([1 0]);
                return;
            end
            assert(isa(from, 'double') && ismatrix(from) && size(from, 1) == 3);
            from = obj.voxels(from);
            assert(isa(to, 'double') && ismatrix(to) && size(to, 1) == 3);
            to = obj.voxels(to);
            [from, to] = VoxelMap.same_cols(from, to);
            if nargin < 4 || isempty(min_val)
                min_val = -inf;
            end
            assert(isa(min_val, 'double') && isscalar(min_val));
            if nargin < 5 || isempty(max_val)
                max_val = obj.occupied_threshold;
            end
            assert(isa(max_val, 'double') && isscalar(max_val));
            
            [vox, val] = voxel_map('trace_lines', obj.map_id, from, to, min_val, max_val);
            
            x = obj.voxel_centers(vox);
        end
        function [x, val] = get_voxels(obj, query, min_val, max_val)
            %GET_VOXELS
            %
            % [x, val] = get_voxels(obj)
            % [x, val] = get_voxels(obj, [], min_val, max_val)
            % [x, val] = get_voxels(obj, query)
            % [x, val] = get_voxels(obj, query, min_val, max_val)
            % Preproc inputs.
            if nargin < 2
                query = [];
            end
            assert(isempty(query) || (isa(query, 'double') && ismatrix(query) && size(query, 1) == 3));
            if nargin < 3 || isempty(min_val)
                if isempty(query)
                    min_val = obj.occupied_threshold;
                else
                    % No default (no filtering) if query is used.
                    min_val = nan;
                end
            end
            assert(isa(min_val, 'double') && isscalar(min_val));
            if nargin < 4 || isempty(max_val)
                max_val = nan;
            end
            assert(isa(max_val, 'double') && isscalar(max_val));
            % Get voxels from map.
            if ~isempty(query)    
                query = obj.voxels(query);
                [vox, val] = voxel_map('get_voxels', obj.map_id, query);
            else
                [vox, val] = voxel_map('get_voxels', obj.map_id);
            end
            % Filter according to min/max values.
            if ~isnan(min_val)
                vox = vox(:, val >= min_val);
                val = val(val >= min_val);
            end
            if ~isnan(max_val)
                vox = vox(:, val <= max_val);
                val = val(val <= max_val);
            end
            x = obj.voxel_centers(vox);
        end
        function update_lines(obj, from, to)
            %UPDATE_LINES
            %
            % update_lines(obj, from, to)
            if isempty(from) || isempty(to)  % No voxels to update.
                return;
            end
            assert(isa(from, 'double') && ismatrix(from) && size(from, 1) == 3);
            assert(isa(to, 'double') && ismatrix(to) && size(to, 1) == 3);
            assert(size(from, 2) == 1 || size(to, 2) == 1 || size(from, 2) == size(to, 2));
            from = obj.voxels(from);
            to = obj.voxels(to);
            [from, to] = VoxelMap.same_cols(from, to);
            voxel_map('update_lines', obj.map_id, from, to, obj.free_update, obj.hit_update);
        end
        function update_voxels(obj, x, val)
            %UPDATE_VOXELS
            %
            % update_voxels(obj, x, val)
            if isempty(x)  % No voxels to update.
                return;
            end
            assert(isa(x, 'double') && ismatrix(x) && size(x, 1) == 3);
            assert(isa(val, 'double') && isvector(val) && numel(val) == size(x, 2));
            vox = obj.voxels(x);
            voxel_map('update_voxels', obj.map_id, vox, val);
        end
        function set_voxels(obj, x, val)
            %SET_VOXELS
            %
            % set_voxels(obj, x, val)
            if isempty(x)  % No voxels to update.
                return;
            end
%             assert(isa(x, 'double') && ismatrix(x) && size(x, 1) == 3);
%             assert(isa(val, 'double') && ismatrix(val) && size(val, 1) == 1);
            assert(ismatrix(x) && size(x, 1) == 3);
            assert(ismatrix(val) && size(val, 1) == 1);
            x = double(x);
            val = double(val);
            vox = obj.voxels(x);
            [vox, val] = VoxelMap.same_cols(vox, val);
            voxel_map('set_voxels', obj.map_id, vox, val);
        end
        function collapse(obj, min_val, max_val)
            if nargin < 2 || isempty(min_val)
                min_val = -inf;
            end
            if nargin < 3 || isempty(max_val)
                max_val = obj.occupied_threshold;
            end
            voxel_map('collapse', obj.map_id, min_val, max_val);
        end
        function expand(obj, min_val, max_val)
            if nargin < 2 || isempty(min_val)
                min_val = obj.occupied_threshold;
            end
            if nargin < 3 || isempty(max_val)
                max_val = inf;
            end
            voxel_map('expand', obj.map_id, min_val, max_val);
        end
        function merge(obj)
            voxel_map('merge', obj.map_id);
        end
        function sobj = saveobj(obj)
            %SAVEOBJ Save internal representation from mex library.
            %
            % sobj = saveobj(obj)
            sobj.voxel_size = obj.voxel_size;
            sobj.free_update = obj.free_update;
            sobj.hit_update = obj.hit_update;
            sobj.occupied_threshold = obj.occupied_threshold;
            % TODO: Level missing.
            [sobj.x, sobj.val] = get_voxels(obj, [], nan, nan);
        end
        function c = copy(obj)
            %COPY Return a deep copy.
            %
            % c = obj.copy();
            c = obj.loadobj(obj.saveobj());
        end
    end
    methods (Static)
        function obj = loadobj(sobj)
            %LOADOBJ Load internal representation to mex library.
            %
            % obj = loadobj(sobj)
            obj = VoxelMap(sobj.voxel_size, sobj.free_update, ...
                           sobj.hit_update, sobj.occupied_threshold);
            % TODO: Level missing.
            obj.set_voxels(sobj.x, sobj.val);
        end
        function varargout = same_cols(varargin)
            nc = cellfun(@(x) size(x, 2), varargin);
            assert(max(nc) >= 1);
            assert(all(nc == 1 | nc == max(nc)));
            varargout = cellfun(@(x) repmat(x, [1 max(nc)/size(x, 2)]), varargin, ...
                'UniformOutput', false);
        end
    end
end
