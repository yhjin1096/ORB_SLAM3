#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

// Rotation matrix to quaternion conversion
Quaterniond rotationMatrixToQuaternion(const Matrix3d& R) {
    return Quaterniond(R);
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        cerr << "Usage: " << argv[0] << " <input_kitti_file> <output_tum_file> [times_file]" << endl;
        cerr << "Example: " << argv[0] << " ../results/05.txt ../results/05_tum.txt [../results/05_times.txt]" << endl;
        cerr << "If times_file is not provided, it will be auto-detected from input file name." << endl;
        return 1;
    }

    string input_file = argv[1];
    string output_file = argv[2];
    string times_file;
    
    // Auto-detect times file if not provided
    if (argc == 4) {
        times_file = argv[3];
    } else {
        // Extract directory and base name from input file
        size_t last_slash = input_file.find_last_of("/");
        size_t last_dot = input_file.find_last_of(".");
        if (last_dot != string::npos && last_slash != string::npos) {
            times_file = input_file.substr(0, last_slash + 1) + 
                        input_file.substr(last_slash + 1, last_dot - last_slash - 1) + "_times.txt";
        } else if (last_dot != string::npos) {
            times_file = input_file.substr(0, last_dot) + "_times.txt";
        } else {
            times_file = input_file + "_times.txt";
        }
    }

    // Read timestamps
    vector<double> timestamps;
    ifstream ftimes(times_file);
    if (!ftimes.is_open()) {
        cerr << "Error: Cannot open times file: " << times_file << endl;
        return 1;
    }
    
    string time_line;
    while (getline(ftimes, time_line)) {
        if (time_line.empty()) continue;
        double t;
        istringstream tss(time_line);
        if (tss >> t) {
            timestamps.push_back(t);
        }
    }
    ftimes.close();
    
    cout << "Loaded " << timestamps.size() << " timestamps from " << times_file << endl;

    ifstream fin(input_file);
    if (!fin.is_open()) {
        cerr << "Error: Cannot open input file: " << input_file << endl;
        return 1;
    }

    ofstream fout(output_file);
    if (!fout.is_open()) {
        cerr << "Error: Cannot open output file: " << output_file << endl;
        fin.close();
        return 1;
    }

    fout << fixed << setprecision(6);
    
    string line;
    int line_count = 0;

    cout << "Converting KITTI format to TUM format..." << endl;

    while (getline(fin, line)) {
        if (line.empty()) continue;

        // Check if we have enough timestamps
        if (line_count >= static_cast<int>(timestamps.size())) {
            cerr << "Warning: Not enough timestamps. Poses: " << line_count + 1 
                 << ", Timestamps: " << timestamps.size() << endl;
            break;
        }

        // KITTI format: 12 values representing 4x4 transformation matrix
        // r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        istringstream iss(line);
        double values[12];
        
        bool valid = true;
        for (int i = 0; i < 12; i++) {
            if (!(iss >> values[i])) {
                valid = false;
                break;
            }
        }

        if (!valid) {
            cerr << "Warning: Invalid line " << line_count + 1 << ", skipping..." << endl;
            continue;
        }

        // Extract rotation matrix (3x3) and translation (3x1)
        Matrix3d R;
        R << values[0], values[1], values[2],
             values[4], values[5], values[6],
             values[8], values[9], values[10];

        Vector3d t(values[3], values[7], values[11]);

        // Convert rotation matrix to quaternion
        Quaterniond q = rotationMatrixToQuaternion(R);
        q.normalize();

        // Get timestamp from the loaded timestamps
        double timestamp = timestamps[line_count];

        // Write in TUM format: timestamp x y z qx qy qz qw
        fout << timestamp << " "
             << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        line_count++;
    }

    fin.close();
    fout.close();

    cout << "Conversion complete!" << endl;
    cout << "Processed " << line_count << " poses" << endl;
    cout << "Output saved to: " << output_file << endl;

    return 0;
}

