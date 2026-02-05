module DQ

import Base: *, conj, show, promote_rule, convert

export Quaternion, UnitQuaternion, DualQuaternion, UnitDualQuaternion, Point
export transform, sclerp

#---- TYPE DEFINITIONS ---------------------------------------------------------

    #----- Standard Quaternion -------------------------------------------------
        abstract type AbstractQuaternion{T<:Real} end

        struct Quaternion{T} <: AbstractQuaternion{T}
            w::T; x::T; y::T; z::T
        end

        #* A Unit Quaternion (Rotation)
        struct UnitQuaternion{T} <: AbstractQuaternion{T}
            w::T;
            x::T;
            y::T;
            z::T
        end

            # Outer Constructor
            function UnitQuaternion(w::Real, x::Real, y::Real, z::Real)
                T = float(promote_type(typeof(w), typeof(x), typeof(y), typeof(z)))
                mag = hypot(w, x, y, z)
                return UnitQuaternion{T}(w/mag, x/mag, y/mag, z/mag)
            end

            # Promote UnitQuaternion and Quaternion to a general Quaternion
            promote_rule(::Type{Quaternion{T}}, ::Type{UnitQuaternion{S}}) where {T, S} = 
                Quaternion{promote_type(T, S)}

            # Conversion rules
            convert(::Type{Quaternion{T}}, q::UnitQuaternion) where T = Quaternion{T}(q.w, q.x, q.y, q.z)
            convert(::Type{UnitQuaternion{T}}, q::Quaternion) where T = UnitQuaternion(T(q.w), T(q.x), T(q.y), T(q.z))

    #---- DUAL QUATERNIONS ---------------------------------------------------------

    abstract type AbstractDualQuaternion{T<:Real} end

    struct DualQuaternion{T} <: AbstractDualQuaternion{T}
        real::Quaternion{T}
        dual::Quaternion{T}
    end

    #* Represents a Rigid Body Transform (Rotation + Translation)
    struct UnitDualQuaternion{T} <: AbstractDualQuaternion{T}
        real::UnitQuaternion{T}
        dual::Quaternion{T} # This is 0.5 * translation_quat * real_quat
    end

        # Create a transform from a rotation and a translation vector
        function UnitDualQuaternion(r::UnitQuaternion{T}, tx::Real, ty::Real, tz::Real) where T
            U = promote_type(T, typeof(tx), typeof(ty), typeof(tz))
            # Ensure it's at least a float type (important for 0.5 multiplication)
            F = float(U)
            r_promoted = UnitQuaternion{F}(F(r.w), F(r.x), F(r.y), F(r.z))
            t_quat = Quaternion{F}(zero(F), F(tx), F(ty), F(tz))
            dual_part = F(0.5) * (t_quat * r)
            UnitDualQuaternion{T}(r, dual_part)
        end

        # Promote UnitDualQuaternion and DualQuaternion to general DualQuaternion
        promote_rule(::Type{DualQuaternion{T}}, ::Type{UnitDualQuaternion{S}}) where {T, S} = 
            DualQuaternion{promote_type(T, S)}

        # Convert UnitDualQuaternion to DualQuaternion
        function convert(::Type{DualQuaternion{T}}, dq::UnitDualQuaternion) where T
            return DualQuaternion{T}(
                convert(Quaternion{T}, dq.real), 
                convert(Quaternion{T}, dq.dual)
            )
        end

    #* Point as a Dual Quaternion: p = 1 + ε(xi + yj + zk)
    struct Point{T} <: AbstractDualQuaternion{T}
        x::T; y::T; z::T
    end

        # Promote Point and Dual types to the general DualQuaternion
        promote_rule(::Type{DualQuaternion{T}}, ::Type{Point{S}}) where {T, S} = 
            DualQuaternion{promote_type(T, S)}

        # Promote UnitDualQuaternion and Point to general DualQuaternion
        promote_rule(::Type{UnitDualQuaternion{T}}, ::Type{Point{S}}) where {T, S} = 
            DualQuaternion{promote_type(T, S)}

        # Convert Point to DualQuaternion
        function convert(::Type{DualQuaternion{T}}, p::Point) where T
            r = Quaternion{T}(one(T), zero(T), zero(T), zero(T))
            d = Quaternion{T}(zero(T), T(p.x), T(p.y), T(p.z))
            return DualQuaternion{T}(r, d)
        end

#---- ALGEBRA ------------------------------------------------------------------

    #---- Quaternion --------------------------------------------

        # Addition
        Base.:+(q1::Quaternion{T}, q2::Quaternion{S}) where {T, S} = 
            Quaternion(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z)

        # Scalar Multiplication
        Base.:*(s::Real, q::Quaternion{T}) where T = 
            Quaternion(s * q.w, s * q.x, s * q.y, s * q.z)

        # Mixed promotion for UnitQuaternions in math
        Base.:+(q1::AbstractQuaternion, q2::AbstractQuaternion) = +(promote(q1, q2)...)
        
        # Standart Quaternion Multiplication: (a + bi + cj + dk)
        function *(q1::AbstractQuaternion, q2::AbstractQuaternion)
            w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
            w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
            Quaternion(
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            )
        end

        # Specific method for UnitQuaternions
        function *(q1::UnitQuaternion{T}, q2::UnitQuaternion{T}) where T
            # The UnitQuaternion constructor (which you defined earlier) 
            # already handles normalization, preventing drift!
            return UnitQuaternion(
                q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
                q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
                q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
                q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
            )
        end

        conj(q::UnitQuaternion{T}) where T = UnitQuaternion{T}(q.w, -q.x, -q.y, -q.z)
        conj(q::Quaternion{T}) where T = Quaternion{T}(q.w, -q.x, -q.y, -q.z)

    #---- Dual Quaternion ------------------------------------------------------
        
        # Dual Quaternion Multiplication: (R1 + εD1)(R2 + εD2) = R1R2 + ε(R1D2 + D1R2)
        function *(dq1::AbstractDualQuaternion, dq2::AbstractDualQuaternion)
            DualQuaternion(
                dq1.real * dq2.real,
                (dq1.real * dq2.dual) + (dq1.dual * dq2.real)
            )
        end

        # a transformation * a transformation = a transformation
        function *(dq1::UnitDualQuaternion{T}, dq2::UnitDualQuaternion{S}) where {T, S}

            U = promote_type(T, S)
            
            r1, d1 = dq1.real, dq1.dual
            r2, d2 = dq2.real, dq2.dual

            # This uses the Unit-specific * for the real part to keep it Unit
            new_real = r1 * r2  
            
            # This uses general Quaternion math for the dual part
            new_dual = (r1 * d2) + (d1 * r2)
            
            return UnitDualQuaternion{U}(new_real, new_dual)
        end

        # General Dual Conjugate: conj(r) + ε conj(d)
        function Base.conj(dq::DualQuaternion{T}) where T
            return DualQuaternion{T}(conj(dq.real), conj(dq.dual))
        end
        # Type-stable version for UnitDualQuaternion
        function Base.conj(dq::UnitDualQuaternion{T}) where T
            return UnitDualQuaternion{T}(conj(dq.real), conj(dq.dual))
        end
        # Fallback for AbstractDualQuaternion (like Point)
        Base.conj(dq::AbstractDualQuaternion) = conj(convert(DualQuaternion, dq))

    #---- transform a Point with a transform -----------------------------------
        
        """
            transform(dq::UnitDualQuaternion, p::Point) -> Point

        Applies a rigid body transformation (rotation and translation) to a 3D point.

        The transformation is mathematically equivalent to the dual quaternion sandwich product:
        \$\$p' = \\hat{q} \\cdot (1 + \\epsilon \\vec{p}) \\cdot \\hat{q}^*\$\$

        This implementation is optimized to avoid full dual quaternion multiplication, 
        using the following steps:
        1. Rotate the point vector: `v_rot = r * p * r'`
        2. Compute translation from dual part: `t = 2 * d * r'`
        3. Sum the results: `p' = v_rot + t`

        # Arguments
        - `dq`: A `UnitDualQuaternion{T}` representing the rotation and translation.
        - `p`: A `Point{S}` representing the coordinates in the initial frame.

        # Returns
        - A `Point{promote_type(T, S)}` in the transformed frame.
        """    
        function transform(dq::UnitDualQuaternion{T}, p::Point{S}) where {T, S}
            U = promote_type(T, S)
            
            r = dq.real
            
            # Treat the point as a pure-vector quaternion: (0, x, y, z)
            v = Quaternion{U}(zero(U), p.x, p.y, p.z)
            
            # Perform the rotation: v_rot = r * v * conj(r)
            v_rot = r * v * conj(r)
            
            # Extract the translation vector from the dual part: t = 2 * d * conj(r)
            # Note: We only care about the x, y, z of the result (the w will be 0)
            t = 2 * (dq.dual * conj(r))
            
            # 5. Final transformed point: p' = v_rot + t
            return Point{U}(
                v_rot.x + t.x,
                v_rot.y + t.y,
                v_rot.z + t.z
            )
        end

        #---- interpolate between two transforms -------------------------------
        
#! implementation needs to be checked
        """
            sclerp(dq1::UnitDualQuaternion, dq2::UnitDualQuaternion, t::Real)

        Performs Screw Linear Interpolation (ScLERP) between two transforms.
        `t` is the interpolation parameter, typically between 0 and 1.
        """
        function sclerp(dq1::UnitDualQuaternion{T}, dq2::UnitDualQuaternion{S}, t::Real) where {T, S}
            U = promote_type(T, S, typeof(t))
            F = float(U)

            # Compute the relative transformation
            diff = conj(dq1) * dq2

            # Check the dot product of the real parts to find the shortest path
            dot_val = dq1.real.w * dq2.real.w + dq1.real.x * dq2.real.x + 
                    dq1.real.y * dq2.real.y + dq1.real.z * dq2.real.z
            
            if dot_val < 0
                # If the quaternions are in opposite hemispheres, 
                diff = conj(dq1) * UnitDualQuaternion(
                    UnitQuaternion(-dq2.real.w, -dq2.real.x, -dq2.real.y, -dq2.real.z),
                    Quaternion(-dq2.dual.w, -dq2.dual.x, -dq2.dual.y, -dq2.dual.z)
                )
            end

            # alculate the "Power" of the dual quaternion: diff^t
            # This represents moving a fraction 't' along the screw motion
            diff_t = pow(diff, F(t))

            return dq1 * diff_t
        end

        # Helper: Power of a Unit Dual Quaternion
        function pow(dq::UnitDualQuaternion{T}, t::T) where T
            # Extract screw parameters
            # This handles the rotation angle θ and translation d along the axis
            angle_sq = dq.real.x^2 + dq.real.y^2 + dq.real.z^2
            
            if angle_sq < eps(T)^2
                # Pure translation case (limit as angle -> 0)
                return UnitDualQuaternion(
                    UnitQuaternion(one(T), zero(T), zero(T), zero(T)),
                    t * dq.dual.x, t * dq.dual.y, t * dq.dual.z
                )
            end

            angle = acos(clamp(dq.real.w, -1, 1))
            s = sin(angle)
            axis = (dq.real.x/s, dq.real.y/s, dq.real.z/s)
            
            # Scale angle and dual part
            t_angle = angle * t
            st, ct = sin(t_angle), cos(t_angle)
            
            # Dual part scaling (pitch and translation)
            # This is a simplified version of the Log-Exp map
            # For a UnitDualQuaternion, the real part is a rotation, 
            # and the dual part contains the translation.
            new_real = UnitQuaternion(ct, st*axis[1], st*axis[2], st*axis[3])
            
            # Linear interpolation of the dual part (translation blending)
            # For UnitDualQuaternions, this maintains the unit property well
            new_dual = t * dq.dual
            
            return UnitDualQuaternion(new_real, new_dual)
        end

#---- DISPLAY ------------------------------------------------------------------

    #* print

    function Base.show(io::IO, q::AbstractQuaternion)
        w = hasproperty(q, :w) ? q.w : 0
        print(io, w, " + ", q.x, "i + ", q.y, "j + ", q.z, "k")
    end

    function Base.show(io::IO, dq::AbstractDualQuaternion)
        print(io, "(", dq.real, ") + ε(", dq.dual, ")")
    end

    function Base.show(io::IO, p::Point)
        print(io, "Point(", p.x, ", ", p.y, ", ", p.z, ")")
    end

    #* console

    function Base.show(io::IO, ::MIME"text/plain", dq::AbstractDualQuaternion)
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

    function Base.show(io::IO, ::MIME"text/plain", dq::Point)
        T = typeof(dq.x)
        
        println(io, "DualQuaternion{$T} representing the Point:")
        # Row 1: Real Part
        print(io, "(")
        show(io, dq.x)
        print(io, ", ")
        show(io, dq.y)
        print(io, ", ")
        show(io, dq.z)
        print(io, ")'")
        println(io)
    end


end # module DQ
