function [t,x] = ackermann_readlog(out,varname)
try
    obj=out.get(varname);
catch
    obj=evalin('base',varname);
end
if isa(obj,'timeseries')
    t=obj.Time(:); x=squeeze(obj.Data);
elseif isstruct(obj) && isfield(obj,'time')
    t=obj.time(:);x=obj.signals.values;
else
    error('Unsupported logged format of %s',varname);
end
if isvector(x)
    x=x(:);
elseif size(x,1)~=numel(t) && size(x,2)==numel(t)
    x=x.';
end
end
