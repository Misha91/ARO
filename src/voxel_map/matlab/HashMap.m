classdef HashMap < handle
    %HASHMAP Vector to scalar hash map

    properties
        map_id
    end
    
    methods
        function obj = HashMap()
            %HASHMAP
            %
            % obj = HashMap()
            obj.map_id = hash_map('create');
        end
        function delete(obj)
            hash_map('delete', obj.map_id);
            obj.map_id = [];
        end
        function clear(obj)
            %CLEAR Clear map
            %
            % clear(obj)
            hash_map('clear', obj.map_id);
        end
        function val = is(obj, key)
            %IS Contains keys?
            %
            % val = is(obj, key)
            assert(nargin == 2);
            assert(isa(key, 'double') && ismatrix(key));
            val = hash_map('is', obj.map_id, key);
        end
        function varargout = get(obj, varargin)
            %GET Get values for keys
            %
            % val = get(obj, key)
            % [key, val] = get(obj)
            if numel(varargin) >= 1
                assert(isa(varargin{1}, 'double') && ismatrix(varargin{1}));
            end
            [varargout{1:nargout}] = hash_map('get', obj.map_id, varargin{:});
        end
        function set(obj, key, val)
            %SET Set values for keys
            %
            % set(obj, key, val)
            assert(isa(key, 'double') && ismatrix(key));
            assert(isa(val, 'double') && ismatrix(val));
            assert(size(key, 1) == size(val, 1));
            hash_map('set', obj.map_id, key, val);
        end
        function sobj = saveobj(obj)
            %SAVEOBJ Save internal representation from mex library.
            %
            % sobj = saveobj(obj)
            [sobj.key, sobj.val] = get(obj);
        end
    end
    methods (Static)
        function obj = loadobj(sobj)
            %LOADOBJ Load internal representation to mex library.
            %
            % obj = loadobj(sobj)
            obj = HashMap();
            set(obj, sobj.key, sobj.val);
        end
    end
end
