using Test
using DQ

@testset "Dual Quaternion Kinematics" begin

    @testset "Rotations" begin
        # 90 degrees around Z axis
        s2 = 1/sqrt(2)
        r90z = DQ.UnitQuaternion(s2, 0, 0, s2)
        p = DQ.Point(1.0, 0.0, 0.0)
        
        # Identity check
        id = DQ.UnitQuaternion(1, 0, 0, 0)
        @test DQ.transform(DQ.UnitDualQuaternion(id, 0,0,0), p).x ≈ 1.0
        
        # Rotation check: (1,0,0) rotated 90 deg around Z -> (0,1,0)
        p_rot = DQ.transform(DQ.UnitDualQuaternion(r90z, 0,0,0), p)
        @test p_rot.x ≈ 0.0 atol=1e-15
        @test p_rot.y ≈ 1.0
    end

    @testset "Translations" begin
        r_id = DQ.UnitQuaternion(1, 0, 0, 0)
        T = DQ.UnitDualQuaternion(r_id, 5.0, -2.0, 0.0)
        p = DQ.Point(0.0, 0.0, 0.0)
        
        p_new = DQ.transform(T, p)
        @test p_new.x ≈ 5.0
        @test p_new.y ≈ -2.0
        @test p_new.z ≈ 0.0
    end

    @testset "Composition" begin
        # Test that T1 * T2 correctly combines transforms
        r_id = DQ.UnitQuaternion(1, 0, 0, 0)
        T1 = DQ.UnitDualQuaternion(r_id, 10.0, 0.0, 0.0) # Move 10 in X
        T2 = DQ.UnitDualQuaternion(r_id, 0.0, 5.0, 0.0)  # Move 5 in Y
        
        T3 = T1 * T2
        p = DQ.Point(0.0, 0.0, 0.0)
        p_final = DQ.transform(T3, p)
        
        @test p_final.x ≈ 10.0
        @test p_final.y ≈ 5.0
    end
    
    @testset "Type Promotion" begin
        # Float32 point transformed by Float64 DQ
        dq = DQ.UnitDualQuaternion(DQ.UnitQuaternion(1.0,0,0,0), 1.0, 0.0, 0.0)
        p = DQ.Point(1.0f0, 0.0f0, 0.0f0)
        res = DQ.transform(dq, p)
        
        @test typeof(res.x) == Float64
    end
#! The testcase is wrong i think
    @testset "Interpolation (ScLERP)" begin
        # Start: Identity
        T1 = UnitDualQuaternion(UnitQuaternion(1,0,0,0), 0, 0, 0)
        # End: Rotate 90 in Z and translate 10 in X
        s2 = 1/sqrt(2)
        T2 = UnitDualQuaternion(UnitQuaternion(s2, 0, 0, s2), 10.0, 0.0, 0.0)
        
        # Midpoint (t=0.5)
        T_mid = sclerp(T1, T2, 0.5)
        
        p = Point(0, 0, 0)
        p_mid = transform(T_mid, p)
        
        # Expect: 45 degree rotation and 5.0 translation in X
        @test p_mid.x ≈ 5.0
        # A 45 deg rotation of (0,0,0) is still (0,0,0), 
        # so we only see the translation.
        @test T_mid.real.w ≈ cos(π/8) # cos(22.5 deg)
    end
end