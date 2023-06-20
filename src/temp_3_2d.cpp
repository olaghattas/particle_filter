//
// Created by olagh on 6/15/23.
//

Eigen::Vector2d projectPoint(const Eigen::Vector3d& point3D, const Eigen::Matrix3d& cameraMatrix, const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& translationVector)
{
    Eigen::Vector3d transformedPoint = rotationMatrix * point3D + translationVector;
    Eigen::Vector3d homogeneousPoint;
    homogeneousPoint << transformedPoint.x() / transformedPoint.z(), transformedPoint.y() / transformedPoint.z(), 1.0;
    Eigen::Vector3d projectedPoint = cameraMatrix * homogeneousPoint;
    Eigen::Vector2d point2D;
    point2D << projectedPoint.x() / projectedPoint.z(), projectedPoint.y() / projectedPoint.z();
    return point2D;
}

std::vector<Eigen::Vector2d> projectParticles(const std::vector<Particle>& particles, const Eigen::Matrix3d& cameraMatrix, const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& translationVector)
{
    std::vector<Eigen::Vector2d> projectedPoints;
    projectedPoints.reserve(particles.size());
    for (const auto& particle : particles) {
        Eigen::Vector3d point3D(particle.x, particle.y, particle.z);
        Eigen::Vector2d point2D = projectPoint(point3D, cameraMatrix, rotationMatrix, translationVector);
        projectedPoints.push_back(point2D);
    }
    return projectedPoints;
}

int main()
{
    Eigen::Matrix3d cameraMatrix;
    // Set camera matrix values

    Eigen::Matrix3d rotationMatrix;
    // Set rotation matrix values

    Eigen::Vector3d translationVector;
    // Set translation vector values

    std::vector<Particle> particles;
    // Populate the vector of particles

    std::vector<Eigen::Vector2d> projectedPoints = projectParticles(particles, cameraMatrix, rotationMatrix, translationVector);

    for (const auto& point : projectedPoints) {
        std::cout << "Projected 2D point: (" << point.x() << ", " << point.y() << ")" << std::endl;
    }

    return 0;
}
