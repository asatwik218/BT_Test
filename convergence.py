import numpy as np
import kinematics
import math


def Increments_to_Degrees_Left_Motor(Parallel_Angle, Motor_Angle):
    intermediate_res = abs(Motor_Angle-Parallel_Angle)/364.08
    res = 90-intermediate_res if Motor_Angle < Parallel_Angle else 90+intermediate_res
    return res


def Increments_to_Degrees_Right_Motor(Parallel_Angle, Motor_Angle):
    intermediate_res = abs(Motor_Angle-Parallel_Angle)/364.08
    res = 90-intermediate_res if Motor_Angle > Parallel_Angle else 90+intermediate_res
    return res


def Increments_to_Degrees_Two_Motors(Parallel_Angles, Motor_Angles):
    Left_Degrees = Increments_to_Degrees_Left_Motor(Parallel_Angles[0], Motor_Angles[0])
    Right_Degrees = Increments_to_Degrees_Right_Motor(Parallel_Angles[1], Motor_Angles[1])
    return [Left_Degrees, Right_Degrees]


def Convergence_Triangulation(Motor_Degree_Angles):
    """
    Compute d, Theta, d1, d2 from given theta1 and theta2 in degrees.
    
    Parameters:
        theta1_deg (float): angle theta1 in degrees
        theta2_deg (float): angle theta2 in degrees
        
    Returns:
        list: [d, Theta_deg, d1, d2]
    """
    p = 90.0  # given constant

    theta1_deg, theta2_deg = Motor_Degree_Angles[0], Motor_Degree_Angles[1]

    # Convert to radians
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    
    # Intermediate calculations
    k = p / math.sin(math.pi - theta1 - theta2)
    d1 = k * math.sin(theta2)
    d2 = k * math.sin(theta1)
    
    d = math.sqrt((p**2 * 0.25) + d2**2 - p * d2 * math.cos(theta2))
    
    Theta = math.atan((2 * d2 * math.sin(theta2)) / (p - 2 * d2 * math.cos(theta2)))
    Theta_deg = math.degrees(Theta)
    
    return [d, Theta_deg, d1, d2]


def X_Pixel_Offset_To_Motor_Incr(X_Offset, Motor_Increment):
    return Motor_Increment+(X_Offset*(15/306)*364)


def X_Pixel_Offsets_To_Motor_Incr(X_Offsets, Motor_Increments):
    L_Motor_Incr = X_Pixel_Offset_To_Motor_Incr(X_Offsets[0], Motor_Increments[0])
    R_Motor_Incr = X_Pixel_Offset_To_Motor_Incr(X_Offsets[1], Motor_Increments[1])
    return [L_Motor_Incr, R_Motor_Incr]


def Y_Pixel_Offset_To_Degree_Angle(VFOV, Y_Pixel_Offset):
    return VFOV*(Y_Pixel_Offset/256)


def A6_Doosan_Convergence(Cartesian_Pose, Motor_Increments, Parallel_Angles, A6_Rotation_Angle, FOVs, Y_Pixel_Offset, Height_From_Link=-3, Length_From_Axis=84.9):
    def get_sign_comparison(number):
        if number > 0:
            return 1
        elif number < 0:
            return -1
        else:  # number is 0
            return 0
    
    HFOV, VFOV = FOVs[0], FOVs[1]
    Doosan_Pose_Hom_Mat = kinematics.Cartesian_Pose_To_Hom_Mat(Cartesian_Pose)
    Cam_Displacement_Mat = kinematics.Translation_To_Hom_Mat(0, Height_From_Link, Length_From_Axis)
    A6_Rotation_Mat = kinematics.Z_Rotation_To_Hom_Mat(A6_Rotation_Angle)
    d_horizontal, Theta_deg, _, _ = Convergence_Triangulation(Increments_to_Degrees_Two_Motors(Parallel_Angles, Motor_Increments))
    Y_Rotation_Mat = kinematics.Y_Rotation_To_Hom_Mat((abs(Theta_deg)-90)*(-get_sign_comparison(Theta_deg)))
    Y_Offset_Angle_Deg = Y_Pixel_Offset_To_Degree_Angle(VFOV, Y_Pixel_Offset)
    X_Rotation_Mat = kinematics.X_Rotation_To_Hom_Mat(Y_Offset_Angle_Deg)
    d_resultant = d_horizontal/(math.cos(math.radians(Y_Offset_Angle_Deg)))
    d_Translation_Mat = kinematics.Translation_To_Hom_Mat(0, 0, d_resultant)
    
    final_transf_mat = Doosan_Pose_Hom_Mat@Cam_Displacement_Mat@A6_Rotation_Mat@Y_Rotation_Mat@X_Rotation_Mat@d_Translation_Mat
    return final_transf_mat[:, -1]


# def A6_Doosan_Convergence(Cartesian_Pose, Motor_Increments, Parallel_Angles, A6_Rotation_Angle, FOVs, Y_Pixel_Offset, Height_From_Link=-3, Length_From_Axis=84.9):
#     def get_sign_comparison(number):
#         if number > 0:
#             return 1
#         elif number < 0:
#             return -1
#         else:  # number is 0
#             return 0

#     HFOV, VFOV = FOVs[0], FOVs[1]

#     print("\n=== A6_Doosan_Convergence Debug Output ===")

#     # Step 1: Cartesian pose to homogeneous matrix
#     Doosan_Pose_Hom_Mat = kinematics.Cartesian_Pose_To_Hom_Mat(Cartesian_Pose)
#     print("Doosan Pose Homogeneous Matrix:\n", Doosan_Pose_Hom_Mat)

#     # Step 2: Camera displacement from link
#     Cam_Displacement_Mat = kinematics.Translation_To_Hom_Mat(0, Height_From_Link, Length_From_Axis)
#     print("Camera Displacement Matrix:\n", Cam_Displacement_Mat)

#     # Step 3: A6 rotation along X axis
#     A6_Rotation_Mat = kinematics.Z_Rotation_To_Hom_Mat(A6_Rotation_Angle)
#     print("A6 Rotation Matrix (X axis, deg={}):\n".format(A6_Rotation_Angle), A6_Rotation_Mat)

#     # Step 4: Convergence triangulation
#     d_horizontal, Theta_deg, d1, d2 = Convergence_Triangulation(
#         Increments_to_Degrees_Two_Motors(Parallel_Angles, Motor_Increments)
#     )
#     print("d_horizontal =", d_horizontal)
#     print("Theta (deg) =", Theta_deg)
#     print("d1 =", d1, "d2 =", d2)

#     # Step 5: Y rotation matrix
#     Y_rotation_angle = (abs(Theta_deg) - 90) * (-get_sign_comparison(Theta_deg))
#     Y_Rotation_Mat = kinematics.Y_Rotation_To_Hom_Mat(Y_rotation_angle)
#     print("Y Rotation Matrix (deg={}):\n".format(Y_rotation_angle), Y_Rotation_Mat)

#     # Step 6: Pixel offset to degree angle
#     Y_Offset_Angle_Deg = Y_Pixel_Offset_To_Degree_Angle(VFOV, Y_Pixel_Offset)
#     print("Y Pixel Offset Angle (deg) =", Y_Offset_Angle_Deg)

#     # Step 7: X rotation for vertical offset
#     X_Rotation_Mat = kinematics.X_Rotation_To_Hom_Mat(Y_Offset_Angle_Deg)
#     print("X Rotation Matrix (deg={}):\n".format(Y_Offset_Angle_Deg), X_Rotation_Mat)

#     # Step 8: Adjusted translation along Z
#     d_resultant = d_horizontal / (math.cos(math.radians(Y_Offset_Angle_Deg)))
#     print("d_resultant =", d_resultant)

#     d_Translation_Mat = kinematics.Translation_To_Hom_Mat(0, 0, d_resultant)
#     print("Final Translation Matrix:\n", d_Translation_Mat)

#     # Step 9: Final transformation matrix
#     final_transf_mat = (
#         Doosan_Pose_Hom_Mat
#         @ Cam_Displacement_Mat
#         @ A6_Rotation_Mat
#         @ Y_Rotation_Mat
#         @ X_Rotation_Mat
#         @ d_Translation_Mat
#     )
#     print("Final Transformation Matrix:\n", final_transf_mat)

#     # Step 10: Return translation vector
#     final_translation_vector = final_transf_mat[:, -1]
#     print("Final Translation Vector:", final_translation_vector)
#     print("========================================\n")

#     return final_translation_vector