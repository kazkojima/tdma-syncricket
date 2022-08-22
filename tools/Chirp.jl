module Chirp

export Er, alpha_chirp
export genchirp
export chirp_raw, chirp_wav
export Sr
using WAV

"""
Linear chirp which has centre frequency `fc` and chirp constant `α`
"""
Er(τ::Real, fc::Real, α::Real) = sin(2π*fc*τ + α*τ^2)

"""
Chirp constant from central bandwidth `Bw` and pulse length `τ0`
"""
function alpha_chirp(Bw::Real, τ0::Real)
    Bw*π/τ0
end

"""
Generate array for chirp which has start frequency `fs`, chirp constant `α`, pulse length `τ0`, sample rate `rate` and length `len`.
"""
function genchirp(fs::Real, α::Real, τ0::Real, rate::Real, len::Real; ϵ0::Real=0)
    [Int16(floor(0.8*2^15*Er(((i/rate) % τ0)+ϵ0, fs, α))) for i in 0:rate*len-1];
end

"""
Create raw S16_LE chirp file `fname` with chirp which has start frequency `fs`, chirp constant `α`, pulse length `τ0`, sample rate `rate`.
```
chirp_raw("test.raw", 440, 27000, 0.1, 44100, 10)
```
"""
function chirp_raw(fname::String, fs::Real, α::Real, τ0::Real, rate::Real, len::Real)
    open(fname, "w") do of
        write(of, genchirp(fs, α, τ0, rate, len, ϵ0=0.5/rate))
    end
end

"""
Create WAV chirp file `fname` with chirp which has start frequency `fs`, chirp constant `α`, pulse length `τ0`, sample rate `rate`.
```
chirp_wav("test.wav", 440, 27000, 0.1, 44100, 10)
```
"""
function chirp_wav(fname::String, fs::Real, α::Real, τ0::Real, rate::Real, len::Real)
    wavwrite(genchirp(fs, α, τ0, rate, len, ϵ0=0.5/rate), fname, Fs=rate)
end

sgn(x::Real) = (x < 0) ? -1 : 1
rect(x::Real) = (-0.5 <= x <= 0.5) ? 1 : 0

"""
`Sr` is a matched filter function for the linear chirp which has
start frequency `fs`, bandwidth `Bw` and pulse length `τ0`.
"""
function Sr(f::Real, fs::Real, Bw::Real, τ0::Real)
    α = alpha_chirp(Bw, τ0)
    fc = fs + Bw/2
    return rect((f-fc)/abs(Bw))/sqrt(abs(Bw/τ0))*exp((π^2/α*(f-fc)^2-sgn(α)*π/4)im)
end

@doc raw"""
An exapmle matched filter. For example, 4410 data can be given with

```
    Sr1(f::Real)=Sr(f, 4400, 8800, 0.1)
    for i in 0:4410-1
        print(Sr1(10*i+5),",")
    end
```
which are corresponds 0Hz-44.1kHz range of frequency.
Generate chirp files:

```
    chparams = [
     4400 8800 0.1;
    13200 -8800 0.1;
    13200 8800 0.1;
    22000 -8800 0.1;
    22000  8800 0.1;
    30800 -8800 0.1;
     4400 8800 0.2
    13200 -8800 0.2]

    chparams = [
     4400 8800 0.1;
    13200 -8800 0.1;
     8800 8800 0.1;
    17600 -8800 0.1;
    13200 8800 0.1;
    22000 -8800 0.1;
     4400 8800 0.2;
    13200 -8800 0.2]

    for i in 1:size(chparams, 1)
        fs, Bw, τ = chparams[i,:]
        chirp_wav("Sonar/chirps2/chirp$(i).wav", fs, alpha_chirp(Bw, τ), τ, 44100, 10)
    end
```
Generate matched filters:
```
    matched_filters = Vector{Vector{ComplexF64}}(undef, size(chparams, 1))
    for i in 1:size(chparams, 1)
        τ = chparams[i,3]
        n = Int(floor(44100*τ/2))
        matched_filters[i]=[Sr(f/τ, chparams[i,:]...) for f in 0:n-1]
    end

    for i in 1:size(chparams, 1)
        open("../chirpset/filt$(i).cvd", "w") do of
            write(of, matched_filters[i])
        end
    end
```

"""

end
