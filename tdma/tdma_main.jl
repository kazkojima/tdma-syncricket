include("detector.jl")
include("proj2dim.jl")

using PyPlot

ndim = 5
nbin = 6400
nframe = 120
frame_begin = 41
w = spzeros(Float32, nbin, nbin)
times = zeros(Float32, (ndim, nframe))
levels = zeros(Float32, (ndim, nframe))

θ = 0.95f0

pilot_file = "../dataset/output4.dat"
rp = Vector{Float32}(undef, nbin*nframe)
read!(pilot_file, rp)
range_profile = reshape(rp, (nbin, nframe))
base_idx = zeros(Int, nframe)
for i in frame_begin:nframe
    r = range_profile[:,i]
    r = 10.0f0 .^ r;
    r = r .- minimum(r)
    r = r / maximum(r)
    adapt_weight!(w, r)
    oh = one_hot(r)
    detections = detect(oh, θ, w)
    detections_idx = first.(Tuple.(findall(isone, detections)));
    args = arglocalmax(sort(detections_idx), r, gap=10)
    base_idx[i] = args[1]
    #times[1,i] = 0.0f0
    levels[1,i] = range_profile[base_idx[i],i]
end
    
for j in 1:4
    range_profile_file = "../dataset/output$(j-1).dat"
    read!(range_profile_file, rp)
    local range_profile = reshape(rp, (nbin, nframe))
    for i in frame_begin:nframe
        r = range_profile[:,i]
        r = 10.0f0 .^ r;
        r = r .- minimum(r)
        r = r / maximum(r)
        adapt_weight!(w, r)
        oh = one_hot(r)
        detections = detect(oh, θ, w)
        detections_idx = first.(Tuple.(findall(isone, detections)));
        args = arglocalmax(sort(detections_idx), r, gap=10)
        if (length(args) > 1)
            idx = random_select(range_profile[args,i], args)
        else
            idx = args[1]
        end
        idx = idx - base_idx[i]
        if idx < 0
            idx += nbin
        end
        times[j+1,i] = idx/nbin
        levels[j+1,i] = range_profile[idx,i]
    end
end

data = times
proj = random_proj2dim(ndim)
points = zeros(Float32, (2, nbin))
for i in frame_begin:nframe
    points[:,i] = proj * data[:,i]
end

pow = [sqrt(sum(levels[:,i].^2)) for i =frame_begin:nframe]
scatter(points[1,frame_begin:nframe], points[2,frame_begin:nframe], c=pow)

    
    
