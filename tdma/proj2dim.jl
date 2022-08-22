function random_proj2dim(dim=5)
    p1 = randn(dim)
    p2 = randn(dim)
    p1 = p1/sqrt(sum(p1.^2))
    p2 = p2 - sum(p1.*p2)*p1
    p2 = p2/sqrt(sum(p2.^2))
    return transpose(hcat(p1,p2))
end

function random_select(levels::Vector{<:Real}, items::Vector{<:Any})
    @assert (all(levels .> 0) && length(levels) <= length(items)) "levels $(levels) are all positive and not too many"
    dist = levels ./ sum(levels)
    sel = [sum(dist[1:i]) for i=1:length(dist)]
    r = rand(1)
    idx = findfirst(sel .> r)
    return items[idx]
end
