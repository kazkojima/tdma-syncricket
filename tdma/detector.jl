using LinearAlgebra
using SparseArrays

function roi(i, l, n)
    return max(i-l, 1):min(i+l, n)
end

function adapt_weight(profile::Vector{Float32}; train_size=10, guard_size=5)
    n = length(profile)
    weights = spzeros(Float32, n, n)
    for i in 1:n
        weights .= 0
        flip = profile[roi(i, train_size + guard_size, n)]
        kernel = 1.0f0 .- flip./max(flip...)
        guard_begin = train_size - max(train_size - i + 1, 0) + 1
        guard_end = min(length(kernel), guard_begin + 2*guard_size)
        kernel[guard_begin:guard_end] .= 0
        kernel ./= sum(kernel)
        @views w = weights[i, roi(i, train_size + guard_size, n)]
        w = kernel
    end
    return -weights
end

function adapt_weight!(weights::SparseMatrixCSC{Float32, Int}, profile::Vector{Float32}; train_size=10, guard_size=5)
    n = length(profile)
    kernel = zeros(Float32, 2*(train_size + guard_size) + 1)
    weights .= 0
    for i in 1:n
        m = length(roi(i, train_size + guard_size, n))
        k = @view kernel[1:m]
        p = @view profile[roi(i, train_size + guard_size, n)]
        copy!(k, p)
        k .= 1.0f0 .- k ./ maximum(k)
        guard_begin = train_size - max(train_size - i + 1, 0) + 1
        guard_end = min(length(k), guard_begin + 2*guard_size)
        k[guard_begin:guard_end] .= 0
        k ./= -sum(k)
        w = @view weights[i, roi(i, train_size + guard_size, n)]
        copy!(w, k)
    end
    return Nothing
end

function detect(input::Matrix{Float32}, theta::Float32, lateral_weight::SparseMatrixCSC{Float32, Int})
    S = zeros(Float32, size(input))
    d = size(input)[2]
    V = zeros(Float32, size(input)[1])
    for j in 1:d
        Jp = @view input[:,j]
        copy!(V, Jp)
        mul!(V, lateral_weight, V, 1.0f0, 1.0f0)
        idx = findall(V .> theta)
        S[idx, j] .= 1
    end
    return S
end

function one_hot(profile::Vector{Float32}, d=9)
    n = length(profile)
    events = Matrix{Float32}(undef, n, d)
    etrain = zeros(Float32, d)
    for i in 1:n
        etrain .= 0
        period = round(Int, (1 - profile[i])*d) + 1
        if (period <= d)
            etrain[period] = 1.0f0
        end
        events[i,:] = etrain
    end
    return events
end

function arglocalmax(idx::Vector{<:Int}, val::Vector{Float32}; gap=1)
    n = length(idx)
    args = Vector{Int}([])
    if (n == 0)
        return args
    end
    s = idx[1]
    group = Vector{Int}([])
    for i in 1:n
        if (idx[i] > s + gap)
            push!(args, group[argmax(val[group])])
            group = Vector{Int}([])
        end
        push!(group, idx[i])
        s = idx[i]
    end
    push!(args, group[argmax(val[group])])
    return args
end

function test(range_profile::Matrix{Float32})
    n = size(range_profile)[1]
    w = spzeros(Float32, n, n)
    for i in 41:120
        profile = range_profile[:,i]
        r = 10.0f0 .^ profile;
        r = r .- minimum(r);
        r = r ./ maximum(r);
        #w = adapt_weight(r);
        adapt_weight!(w, r);
        oh = one_hot(r);
        detections = detect(oh, 0.95f0, w);
        detected_idx = first.(Tuple.(findall(isone, detections)));
        args = arglocalmax(sort(detected_idx), r, gap=10);
        println("$(i): $(args)")
    end
end

#using PyPlot
#using Profile
#
#rp = Vector{Float32}(undef,filesize("../dataset/output4.dat") ÷ sizeof(Float32));
#read!("../dataset/output4.dat", rp);
#range_profile = reshape(rp, (6400, 120));
#
#r = range_profile[:,106]
#r = r .- minimum(r);
#r = r / maximum(r);
#w = spzeros(Float32, 6400, 6400)
#adapt_weight!(w, r);
#oh = one_hot(r)
#detections = detect(oh, 0.95f0, w);
#Profile.clear_malloc_data()
#oh = one_hot(r)
#detections = detect(oh, 0.95f0, w);
