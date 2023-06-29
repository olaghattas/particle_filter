//
// Created by olagh on 6/15/23.
//

//Helper file that i store function that might be useful later
// Function to perform the projection
//std::vector<Eigen::Vector2d> projectParticlesto2D(const std::vector<Particle> &particles, const Eigen::Matrix3d &cameraMatrix)
//{   std::vector<Eigen::Vector2d> projectedPoints;
//    projectedPoints.reserve(particles.size());
//
//    for (const auto& particle : particles) {
//        Eigen::Vector2d point2D;
//        point2D << (cameraMatrix(0, 0) * particle.x + cameraMatrix(0, 1) * particle.y + cameraMatrix(0, 2) * particle.z) / (cameraMatrix(2, 0) * particle.x + cameraMatrix(2, 1) * particle.y + cameraMatrix(2, 2) * particle.z),
//                (cameraMatrix(1, 0) * particle.x + cameraMatrix(1, 1) * particle.y + cameraMatrix(1, 2) * particle.z) / (cameraMatrix(2, 0) * particle.x + cameraMatrix(2, 1) * particle.y + cameraMatrix(2, 2) * particle.z);
//        projectedPoints.push_Particle> &particlesback(point2D);
//    }
//    return projectedPoints;

//}

//std::vector<Eigen::Vector2d> projectParticles(const std::vector<Particle>& particles, const Eigen::Matrix3d& cameraMatrix, const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& translationVector)
//{
//    std::vector<Eigen::Vector2d> projectedPoints;
//    projectedPoints.reserve(particles.size());
//    for (const auto& particle : particles) {
//        Eigen::Vector3d point3D(particle.x, particle.y, particle.z);
//        Eigen::Vector2d point2D = projectPoint(point3D, cameraMatrix, rotationMatrix, translationVector);
//        projectedPoints.push_back(point2D);
//    }
//    return projectedPoints;
//}
//
//int main()
//{
//    Eigen::Matrix3d cameraMatrix;
//    // Set camera matrix values
//
//    Eigen::Matrix3d rotationMatrix;
//    // Set rotation matrix values
//
//    Eigen::Vector3d translationVector;
//    // Set translation vector values
//
//    std::vector<Particle> particles;
//    // Populate the vector of particles
//
//    std::vector<Eigen::Vector2d> projectedPoints = projectParticles(particles, cameraMatrix, rotationMatrix, translationVector);
//
//    for (const auto& point : projectedPoints) {
//        std::cout << "Projected 2D point: (" << point.x() << ", " << point.y() << ")" << std::endl;
//    }
//
//    return 0;
//}
