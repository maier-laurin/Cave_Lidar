module DualQuaternions

using RecipesBase
#in order to not overite them for all the other types we need to import the functions first
import Base: +, -, *, /, conj, show, promote_rule

export Quaternion, DualQuaternion, 
       dual_num_conj, dual_conj,
       normalize, rotation_matrix, translation_vector,
       slerp, sclerp, transform, point_at, get_point
#---- TYPE DEFINITIONS ---------------------------------------------------------

    struct Quaternion{T<:Real}
        w::T
        x::T
        y::T
        z::T
    end

    struct DualQuaternion{T<:Real}
        real::Quaternion{T}
        dual::Quaternion{T}
    end

    #---- Constructors ---------------------------------------------------------

    #* for promoting different real types
    function Quaternion(w::Real, x::Real, y::Real, z::Real)
        T = promote_type(typeof(w), typeof(x), typeof(y), typeof(z))
        Quaternion(T(w), T(x), T(y), T(z))
    end

    #* for (scalar, vector) tuple input
    function Quaternion(w::T, v::NTuple{3, T}) where T
        Quaternion(w, v[1], v[2], v[3])
    end

    #* from 8 individual components
    function DualQuaternion(rw::Real, rx::Real, ry::Real, rz::Real, dw::Real, dx::Real, dy::Real, dz::Real)
        T = promote_type(typeof(rw), typeof(rx), typeof(ry), typeof(rz), 
                        typeof(dw), typeof(dx), typeof(dy), typeof(dz))
        r = Quaternion(T(rw), T(rx), T(ry), T(rz))
        d = Quaternion(T(dw), T(dx), T(dy), T(dz))
        return DualQuaternion(r, d)
    end

    #* for rotation Quaternion from roll, pitch and yaw
    """
        Quaternion(roll, pitch, yaw)
    Constructs a unit quaternion from Euler angles (ZYX convention).
    """
    function Quaternion(roll::Real, pitch::Real, yaw::Real)
        # Abbreviations for the various angular functions
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(w, x, y, z)
    end

    #* representing just a point / translation
    """
        DualQuaternion(pos::NTuple{3, Real})
    Constructs a Dual Quaternion representing a pure translation (point in space).
    Formula: dq = 1 + ε(v/2), where v is the vector as a pure quaternion.
    """
    function DualQuaternion(pos::NTuple{3, T}) where T <: Real
        r = Quaternion(one(T), zero(T), zero(T), zero(T)) # Identity rotation
        d = Quaternion(zero(T), pos[1]/2, pos[2]/2, pos[3]/2)
        return DualQuaternion(r, d)
    end

    #* representing just a translation + rotation
    """
        DualQuaternion(rot:Quaternion{Real}, pos::NTuple{3, Real})
    Constructs a Dual Quaternion representing a translation plus rotation.
    Formula: dq = 1 + ε(v/2), where v is the vector as a pure quaternion.
    """
    function DualQuaternion(rot::Quaternion{T} ,pos::NTuple{3, T}) where T <: Real
        d = Quaternion(zero(T), pos[1]/2, pos[2]/2, pos[3]/2)
        return DualQuaternion(rot, d)
    end

    #---- identity and zero elements -------------------------------------------

    #* For Quaternions
    Base.one(::Type{Quaternion{T}}) where T = Quaternion{T}(one(T), zero(T), zero(T), zero(T))
    Base.one(::Type{Quaternion}) = one(Quaternion{Float64})
    Base.one(q::Quaternion{T}) where T = one(Quaternion{T})

    Base.zero(::Type{Quaternion{T}}) where T = Quaternion{T}(zero(T), zero(T), zero(T), zero(T))
    Base.zero(::Type{Quaternion}) = zero(Quaternion{Float64})
    Base.zero(q::Quaternion{T}) where T = zero(Quaternion{T})

    #* For Dual Quaternions
    Base.one(::Type{DualQuaternion{T}}) where T = DualQuaternion(one(Quaternion{T}), zero(Quaternion{T}))
    Base.one(::Type{DualQuaternion}) = one(DualQuaternion{Float64})
    Base.one(dq::DualQuaternion{T}) where T = one(DualQuaternion{T})

    Base.zero(::Type{DualQuaternion{T}}) where T = DualQuaternion(zero(Quaternion{T}), zero(Quaternion{T}))
    Base.zero(::Type{DualQuaternion}) = zero(DualQuaternion{Float64})
    Base.zero(dq::DualQuaternion{T}) where T = zero(DualQuaternion{T})

    #---- representing Points in space -----------------------------------------
    function point_at(x, y, z)
        return DualQuaternion(one(Quaternion), Quaternion(0.0, x, y, z))
    end

#---- DISPLAY LOGIC ------------------------------------------------------------

    function Base.show(io::IO, q::Quaternion)
        print(io, q.w, " + ", q.x, "i + ", q.y, "j + ", q.z, "k")
    end

    function Base.show(io::IO, dq::DualQuaternion)
        print(io, "(", dq.real, ") + ε(", dq.dual, ")")
    end

    function Base.show(io::IO, ::MIME"text/plain", dq::DualQuaternion)
        T = typeof(dq.real.w)
        
        println(io, "DualQuaternion{$T}:")
        # Row 1: Real Part
        print(io, "  Real: ")
        show(io, dq.real)
        println(io)
        # Row 2: Dual Part
        print(io, "  Dual: ")
        show(io, dq.dual)
    end
#---- BASIC ARITHMETIC ---------------------------------------------------------

    #---- Conjugation ----------------------------------------------------------
    Base.conj(q::Quaternion) = Quaternion(q.w, -q.x, -q.y, -q.z)
    #* the dual quaternion has 3 conjugates
    # 1. Standard Quaternion Conjugate (q̅)
    Base.conj(q::Quaternion) = Quaternion(q.w, -q.x, -q.y, -q.z)
    # 2. Dual Number Conjugate (dq*): Flips the dual sign ONLY
    # This is often used in internal DQ math
    dual_num_conj(dq::DualQuaternion) = DualQuaternion(dq.real, -dq.dual)
    # 3. Full Dual Conjugate (dq̅*): Flips dual sign AND conjugates quaternions
    # THIS is whats used for: p' = dq * p * dual_conj(dq)
    dual_conj(dq::DualQuaternion) = DualQuaternion(conj(dq.real), -conj(dq.dual))

    #---- Addition -------------------------------------------------------------
    +(a::Quaternion, b::Quaternion) = Quaternion(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z)
    +(a::DualQuaternion, b::DualQuaternion) = DualQuaternion(a.real + b.real, a.dual + b.dual)

    #---- Additive Inverse -----------------------------------------------------
    -(a::Quaternion) = Quaternion(-a.w, -a.x, -a.y, -a.z)
    -(dq::DualQuaternion) = DualQuaternion(-dq.real, -dq.dual)

    #---- Subtraction ----------------------------------------------------------
    -(a::Quaternion, b::Quaternion) = Quaternion(a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z)
    -(a::DualQuaternion, b::DualQuaternion) = DualQuaternion(a.real - b.real, a.dual - b.dual)

    #---- Scalar Multiplication ------------------------------------------------
    *(s::Real, q::Quaternion) = Quaternion(s*q.w, s*q.x, s*q.y, s*q.z)
    *(s::Real, dq::DualQuaternion) = DualQuaternion(s*dq.real, dq.dual)

    #---- INTERNAL HELPER MATH (NTuples) ---------------------------------------

        #* Dot product
        ⋅(a::NTuple{3, T}, b::NTuple{3, T}) where T = a[1]*b[1] + a[2]*b[2] + a[3]*b[3]

        #* Cross product
        ×(a::NTuple{3, T}, b::NTuple{3, T}) where T = (
            a[2]*b[3] - a[3]*b[2],
            a[3]*b[1] - a[1]*b[3],
            a[1]*b[2] - a[2]*b[1]
        )

        #* Vector/Scalar operations
        *(s::Real, v::NTuple{3, T}) where T = (s * v[1], s * v[2], s * v[3])
        +(v1::NTuple{3, T}, v2::NTuple{3, T}) where T = (v1[1]+v2[1], v1[2]+v2[2], v1[3]+v2[3])

    #---- Multiplication -------------------------------------------------------

        function *(a::Quaternion, b::Quaternion)
            s1, v1 = a.w, (a.x, a.y, a.z)
            s2, v2 = b.w, (b.x, b.y, b.z)
            
            # Quaternion product formula: (s1s2 - v1⋅v2, s1v2 + s2v1 + v1×v2)
            scalar_part = s1 * s2 - (v1 ⋅ v2)
            vector_part = (s1 * v2) + (s2 * v1) + (v1 × v2)
            
            return Quaternion(scalar_part, vector_part)
        end

        function *(a::DualQuaternion, b::DualQuaternion)
            # DQ mult: (r1*r2) + (r1*d2 + d1*r2)ε
            r = a.real * b.real
            d = (a.real * b.dual) + (a.dual * b.real)
            return DualQuaternion(r, d)
        end

    #---- Multiplicative inverse -----------------------------------------------
        function Base.inv(q::Quaternion)
            # q⁻¹ = conj(q) / |q|²
            mag_sq = q.w^2 + q.x^2 + q.y^2 + q.z^2
            c = conj(q)
            return Quaternion(c.w/mag_sq, c.x/mag_sq, c.y/mag_sq, c.z/mag_sq)
        end

        function Base.inv(dq::DualQuaternion)
            # dq⁻¹ = r⁻¹ - ε(r⁻¹ * d * r⁻¹)
            r_inv = inv(dq.real)
            d_part = -(r_inv * dq.dual * r_inv)
            return DualQuaternion(r_inv, d_part)
        end

    #---- Division -------------------------------------------------------------
        /(a::Quaternion, b::Quaternion) = a * inv(b)
        /(a::DualQuaternion, b::DualQuaternion) = a * inv(b)

    #---- NORMALIZATION & TRANSFORMS -------------------------------------------

        function normalize(q::Quaternion)
            mag = hypot(q.w, q.x, q.y, q.z)
            return Quaternion(q.w/mag, q.x/mag, q.y/mag, q.z/mag)
        end

        function rotation_matrix(dq::DualQuaternion)
            qr = dq.real
            w, x, y, z = qr.w, qr.x, qr.y, qr.z
            
            return [
                1-2y^2-2z^2   2x*y-2w*z     2x*z+2w*y;
                2x*y+2w*z     1-2x^2-2z^2   2y*z-2w*x;
                2x*z-2w*y     2y*z+2w*x     1-2x^2-2y^2
            ]
        end

        function translation_vector(dq::DualQuaternion)
            # Formula: t_quat = 2 * dq.dual * conj(dq.real)
            # The result's vector part (x, y, z) is the translation
            t_quat = (dq.dual * conj(dq.real))
            return [2 * t_quat.x, 2 * t_quat.y, 2 * t_quat.z]
        end

    #---- if the dual quaternion should represen a point -----------------------
        function get_point(dq::DualQuaternion)
            t_quat = dq.dual
            return [t_quat.x, t_quat.y, t_quat.z]
        end
#---- ADVANCED FUNCTIONALITY ---------------------------------------------------
    #---- Interpolation --------------------------------------------------------

    """
        slerp(q1::Quaternion, q2::Quaternion, t::Real)
    Spherical Linear Interpolation between two quaternions.
    """
    function slerp(q1::Quaternion, q2::Quaternion, t::Real)
        # Compute the cosine of the angle between the quaternions
        dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
        
        # If the dot product is negative, slerp won't take the shorter path.
        # We fix this by reversing one quaternion.
        if dot < 0.0
            q2 = -q2
            dot = -dot
        end

        # If the quaternions are very close, use linear interpolation to avoid division by zero
        if dot > 0.9995
            return normalize(q1 + t * (q2 - q1))
        end

        theta_0 = acos(clamp(dot, -1.0, 1.0))        # Angle between input vectors
        theta = theta_0 * t        # Angle between q1 and result
        sin_theta = sin(theta)
        sin_theta_0 = sin(theta_0)

        s1 = cos(theta) - dot * sin_theta / sin_theta_0
        s2 = sin_theta / sin_theta_0

        return (s1 * q1) + (s2 * q2)
    end

    """
        sclerp(dq1::DualQuaternion, dq2::DualQuaternion, t::Real)
    Screw Linear Interpolation for Dual Quaternions.
    """
    function sclerp(dq1::DualQuaternion, dq2::DualQuaternion, t::Real)
        # Ensure we take the shortest path
        dot = dq1.real.w*dq2.real.w + dq1.real.x*dq2.real.x + dq1.real.y*dq2.real.y + dq1.real.z*dq2.real.z
        if dot < 0.0
            dq2 = -dq2
        end
        
        # Difference dual quaternion: dq1⁻¹ * dq2
        diff = inv(dq1) * dq2
        
        # Raising a dual quaternion to a power (diff^t) is the standard ScLERP approach
        # For a more robust implementation, we can interpolate the components:
        r_interp = slerp(dq1.real, dq2.real, t)
        
        # Interpolating the dual part is more complex, but a linear approach works 
        # well for small steps or when used in rigid body dynamics:
        d_interp = dq1.dual + t * (dq2.dual - dq1.dual)
        
        return normalize(DualQuaternion(r_interp, d_interp))
    end
end # module