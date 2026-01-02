## 높이 기반 Scale 추정 구현 프로세스 (Post-processing 방식)

### 개요
모든 SLAM 데이터 처리가 완료된 후 (`SLAM.Shutdown()` 이후), post-processing 방식으로 ground plane을 검출하고 scale을 계산하여 trajectory에 적용합니다.

### 1단계: 데이터 수집 및 초기화

**SLAM 종료 후 데이터 수집**
- `SLAM.GetAllKeyFrames()`: 모든 키프레임 가져오기
- `SLAM.GetAllMapPoints()`: 모든 맵 포인트 가져오기
- `GroundPlaneDetector` 초기화 (카메라 높이: 1.65m, minInliers: 30, RANSAC threshold: 0.1m, maxIterations: 1000)

### 2단계: 각 키프레임별 Ground Plane 검출

**로컬 맵 포인트 수집**
- 각 키프레임 `i`에 대해:
  - 현재 키프레임과 최대 20개 이전 키프레임 범위 설정: `startIdx = max(0, i - 20)`
  - 해당 범위의 키프레임들에서 관찰된 모든 map point 수집
  - 예: 키프레임 30 → 키프레임 10~30에서 관찰된 map point 사용

**Ground Point 후보 필터링** (`FilterGroundCandidates`)
- World 좌표계 기준 필터링:
  - 맵 포인트의 Y 좌표가 카메라 Y 좌표보다 0.05m 이상 큰 점만 선택
  - 조건: `pos.y() > cameraCenter.y() + 0.05`

**Ground Plane 검출** (`DetectGroundPlane`)
- RANSAC 평면 피팅:
  - 3점을 랜덤 선택하여 평면 생성
  - 평면 normal이 카메라 Y축과 3도 이내로 평행한지 검증 (cos(3°) ≈ 0.9986)
  - Normal이 양의 Y 방향을 향하도록 조정
  - 최소 30개 이상의 inlier 필요 (거리 < 0.1m)
- Iterative Refinement:
  - 최대 30회 반복
  - 현재 평면의 inlier들을 수집
  - Inlier들로 최소자승법(Least Squares)으로 평면 재피팅
  - 공분산 행렬의 최소 고유값에 해당하는 고유벡터를 normal로 사용
  - 개선이 없으면 종료

### 3단계: Scale Factor 계산

**각 키프레임별 Scale 계산** (`EstimateScale`)
- Ground plane과 카메라 중심 간 거리 계산:
  ```
  slamDistance = |plane.normal · cameraCenter + plane.d|
  ```
- Scale factor 계산:
  ```
  scale = actual_camera_height / slamDistance
  scale = 1.65m / slamDistance
  ```
- 검증 조건:
  - Plane inliers >= 30
  - Plane confidence >= 0.3
  - slamDistance > 0 (유효한 거리)
  - Scale > 0 (유효한 scale)

### 4단계: Scale Factor 필터링

**Median Filter 적용** (`FilterScaleMedian`)
- 모든 키프레임에서 계산된 유효한 scale factor 수집
- Scale factor들을 정렬
- 중앙값(median) 선택:
  - 홀수 개: 중간값
  - 짝수 개: 두 중간값의 평균
- 최종 scale factor로 사용

### 5단계: 궤적에 Scale 적용 및 저장

**Trajectory 저장** (`SaveKeyFrameTrajectoryTUM`)
- 모든 키프레임을 ID 순으로 정렬 (`KeyFrame::lId`)
- 각 키프레임의 translation에 최종 scale factor 적용:
  ```cpp
  Sophus::SE3f Twc = keyframe->GetPoseInverse();
  Eigen::Vector3f t = Twc.translation();
  t *= finalScale;  // Scale 적용
  ```
- TUM 형식으로 파일 저장

### 6단계: 시각화 (선택사항)

**Ground Point 및 Plane 시각화**
- Pangolin viewer를 사용하여 시각화
- 모든 키프레임과 맵 포인트 표시
- 각 키프레임의 ground point (초록색) 및 ground plane (반투명 초록색 사각형) 표시
- 버튼으로 키프레임 간 이동 가능

## 구현 위치

**`mono_scale_kitti.cpp`**:
1. 메인 루프에서 모든 이미지 처리 (`SLAM.TrackMonocular()`)
2. `SLAM.Shutdown()` 호출
3. 모든 키프레임과 맵 포인트 수집
4. 각 키프레임별 ground plane 검출 (로컬 맵 포인트 사용)
5. 각 키프레임별 scale 계산
6. Median filter로 최종 scale 결정
7. Scale이 적용된 trajectory 저장

**`GroundPlaneDetector.cc`**:
- `FilterGroundCandidates()`: World 좌표계 기준 ground point 후보 필터링
- `DetectGroundPlane()`: RANSAC + Iterative refinement로 ground plane 검출
- `FitPlaneRANSAC()`: RANSAC 평면 피팅 (3도 이내 평행 검증)
- `FitPlaneFromInliers()`: 최소자승법으로 평면 재피팅
- `EstimateScale()`: Ground plane과 카메라 높이로 scale 계산
- `FilterScaleMedian()`: Median filter로 최종 scale 결정

## 주요 특징

1. **Post-processing 방식**: 모든 데이터 처리 후 일괄 처리
2. **로컬 맵 포인트 사용**: 각 키프레임마다 최대 20개 이전 키프레임의 map point 사용
3. **World 좌표계 기준 필터링**: 카메라 Y 좌표보다 큰 점들만 선택
4. **엄격한 평면 검증**: 카메라 Y축과 3도 이내로 평행한 평면만 허용
5. **Iterative Refinement**: RANSAC 후 최대 30회 반복하여 평면 개선
6. **Median Filter**: 모든 키프레임의 scale 중 중앙값 사용
7. **단일 Scale 적용**: 모든 키프레임에 동일한 최종 scale 적용

## 사용 방법

scale 적용 및 keyframe trajectory 저장
```cpp
./run_kitti_scale.sh
```

gt, scale 미적용, scale 적용 결과 trajectory 비교
```cpp
./run_eval_traj.sh
```

gt, scale 적용 결과 평가
```cpp
./run_eval_ape.sh
```