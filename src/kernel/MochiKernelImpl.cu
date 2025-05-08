/* 
 * 了解しました。`MochiKernel_impl.cu` の実装において、Mochi論文のアルゴリズム（特にメッシュ衝突のためのReduction 4）をCUDA/OptiXで効率的に実装するための設計を検討します。

**1. Mochi Reduction 4 のアルゴリズム再確認**

1.  **入力:** 衝突判定を行うメッシュ（またはメッシュ群）。ここでは簡単のため、メッシュAとメッシュBの衝突（またはメッシュAの自己衝突）を考えます。入力データは頂点座標と元の三角形の頂点インデックスです。
2.  **補助三角形生成:** メッシュB（自己衝突の場合はメッシュA）の*各*元の三角形に対し、その3つの辺に垂直な補助三角形を3つ生成します。これらの補助三角形は、元の三角形の面から微小（ε）だけオフセットされます。
3.  **OptiX GAS構築:** メッシュB（自己衝突の場合はメッシュA）の*全ての*ジオメトリ（元の三角形 + 全ての補助三角形）を含むOptiX Acceleration Structure (GAS、実質的にはBVH) を構築します。この際、後でヒットしたプリミティブが元のか補助のかを区別できる情報（SBTオフセットやプリミティブIDの範囲など）を保持する必要があります。
4.  **レイトレーシング:** メッシュAの*各*元の三角形について、その3つの辺に沿ってレイを生成します。（レイの始点は辺の一方の頂点、方向はもう一方の頂点へ、長さは辺の長さ）。
5.  **交差判定:** 生成されたレイを、ステップ3で構築したメッシュB（またはA）のGASに対してトレースします。AnyHitシェーダーを使用して、レイが通過する*全ての*交差を検出します。
6.  **ヒット処理 (AnyHitシェーダー):**
    *   ヒットしたプリミティブのタイプ（元 or 補助）を識別します（SBTデータやプリミティブIDを使用）。
    *   **元プリミティブヒット:** 衝突確定。レイを発射した元の三角形Aのインデックスと、ヒットした元の三角形Bのインデックスのペアを結果として記録します。
    *   **補助プリミティブヒット:** 潜在的な衝突。
        *   交差点Pを計算します。
        *   ヒットした補助三角形が属する*元の*三角形B'の、対応する*元の辺* E' を特定します。
        *   交差点Pが、元の辺E'上に（許容誤差ε'内で）存在するかどうかを判定します。
        *   存在する場合：衝突確定。レイを発射した元の三角形Aのインデックスと、補助三角形が属していた元の三角形B'のインデックスのペアを結果として記録します。
7.  **結果:** 記録された全ての衝突ペアのリスト。自己衝突の場合、(A, A) のような自明な衝突は除外する必要があります。

**2. `MochiKernel_impl.cu` の設計方針**

この設計は、効率性、データの局所性、OptiXの機能を考慮します。

*   **データ構造 (`MochiKernelData_impl`):**
    *   `OptixDeviceContext context`: (グローバルまたは共有が望ましい) OptiXコンテキスト。
    *   `OptixTraversableHandle gasHandle`: ビルドされたGASへのハンドル。
    *   `CUdeviceptr d_gas_output_buffer`: GAS自体のメモリ（解放に必要）。
    *   `CUdeviceptr d_vertices`: 頂点データ（`Point_cu`）。
    *   `CUdeviceptr d_original_indices`: 元の三角形のインデックス（`int3`）。
    *   `CUdeviceptr d_aux_indices`: 生成された補助三角形のインデックス（`int3`）。
    *   `CUdeviceptr d_original_face_indices`: 各元の三角形に対応するMayaのFace Index（`int`）。結果報告用。
    *   `unsigned int numVertices`: 頂点数。
    *   `unsigned int numOriginalTriangles`: 元の三角形の数。
    *   `unsigned int numAuxTriangles`: 補助三角形の数 (`numOriginalTriangles * 3`)。
    *   `unsigned int numTotalPrimitives`: 全プリミティブ数 (`numOriginalTriangles + numAuxTriangles`)。

*   **GAS構築 (`mochiKernelBuild_impl`):**
    *   **補助三角形生成:** CUDAカーネルで実装するのが最も効率的です。入力として`d_vertices`, `d_original_indices`を受け取り、`d_aux_indices`に出力します。
        *   カーネル内では、各元の三角形について法線を計算し、辺ごとに垂直な面を定義して補助三角形の頂点インデックスを計算・格納します（新しい頂点を生成するのではなく、既存の頂点インデックスを使用し、オフセットは概念的なものか、シェーダーで適用します。ただし、論文の図を見ると新しい頂点を生成しているようにも見えます。ここでは既存頂点を使うアプローチをまず考えます。もし精度が必要なら微小オフセットした頂点を実際に生成し `d_vertices` を拡張する必要があるかもしれません）。
    *   **`OptixBuildInput`:** 2つの`OptixBuildInput`を用意します。
        *   Input 0: 元の三角形 (`d_vertices`, `d_original_indices`, `numOriginalTriangles`)。`numSbtRecords = 1`, `sbtIndexOffset = 0`。
        *   Input 1: 補助三角形 (`d_vertices`, `d_aux_indices`, `numAuxTriangles`)。`numSbtRecords = 1`, `sbtIndexOffset = 1` (SBTで区別するため)。 `primitiveIndexOffset = numOriginalTriangles` (AnyHitシェーダーで区別するため)。
    *   **ビルド:** `optixAccelBuild` を呼び出してGASを構築します。Compactionを使うとメモリ効率が良い場合があります。

*   **衝突判定 (`mochiKernelIntersect_impl`):**
    *   **グローバルOptiX設定:** Pipeline, Module, Program Groups, SBTレコードは、理想的にはプラグイン初期化時などに一度だけ作成し、再利用します。`MochiKernelData_impl`にはGASハンドルとバッファポインタのみを持たせます。
    *   **SBT設定:**
        *   RayGenレコード: `__raygen__shootEdgeRays` を指す。
        *   Missレコード: （必要なら）`__miss__miss` を指す。
        *   HitGroupレコード (2つ):
            *   Record 0 (元三角形用): `__anyhit__processHit` を指す。SBTデータとして、関連するバッファ（`d_vertices`, `d_original_indices`, `d_original_face_indices`）へのポインタと、「元タイプ」を示すフラグを含める。
            *   Record 1 (補助三角形用): `__anyhit__processHit` を指す。SBTデータとして、関連するバッファ（`d_vertices`, `d_aux_indices`）へのポインタと、「補助タイプ」を示すフラグ、および補助三角形から元の三角形/辺へのマッピングに必要な情報を含める。
    *   **Launch Parameters (`Params` struct):**
        *   `OptixTraversableHandle targetGas`: 衝突対象のGASハンドル (`handleB->gasHandle`)。
        *   `MochiCollisionPairPOD* collisionBuffer`: 結果を格納するGPUバッファへのポインタ。
        *   `unsigned int* collisionCounter`: 結果の数をカウントするアトミックカウンターへのポインタ。
        *   `Point_cu* meshAVertices`: レイを発射するメッシュAの頂点バッファ (`handleA->d_vertices`)。
        *   `int3* meshAIndices`: レイを発射するメッシュAの元の三角形インデックス (`handleA->d_original_indices`)。
        *   `unsigned int meshANumOriginalTriangles`: メッシュAの元の三角形数 (`handleA->numOriginalTriangles`)。
        *   (自己衝突の場合、`targetGas`は`handleA->gasHandle`になる)。
    *   **GPUバッファ:** 結果格納用の`collisionBuffer`と`collisionCounter`をGPUに確保します。`collisionBuffer`のサイズは、最大衝突数を推定するか、超過分を扱えるように動的にするか、アトミックカウンターの結果に基づいて後でホストにコピーする量を決めます。
    *   **`optixLaunch`:**
        *   `launchDimX = handleA->numOriginalTriangles`
        *   `launchDimY = 3` (各三角形から3本のレイ)
        *   `launchDimZ = 1`
    *   **結果取得:** `collisionCounter`の値をホストにコピーし、その数だけ`collisionBuffer`から結果をホストの`outCollisionPairs`にコピーします。

*   **OptiXプログラム:**
    *   **`__raygen__shootEdgeRays`:**
        *   `optixGetLaunchIndex()` から、処理対象のメッシュAの元の三角形インデックス `triIdx` と辺インデックス `edgeIdx` (0, 1, 2) を取得します。
        *   `Params` からメッシュAの頂点とインデックスを取得します。
        *   `triIdx` に対応する三角形の頂点 `v0, v1, v2` を読み込みます。
        *   `edgeIdx` に基づいて、レイの始点 `origin` と方向 `dir` を決定します (例: edge 0 -> origin=v0, dir=v1-v0)。
        *   レイの`tmin = 0`, `tmax = length(dir)` を設定します。
        *   ペイロードレジスタ `p0` に `triIdx` を格納します。
        *   `optixTrace` を呼び出します。`sbtOffset = 0`, `sbtStride = 2` (ヒットグループが2種類あるため), `missSbtIndex = 0`。
    *   **`__anyhit__processHit`:**
        *   `optixGetSbtDataPointer()` から、ヒットしたジオメトリの種類（元/補助）と関連バッファへのポインタを取得します。
        *   `optixGetPrimitiveIndex()` から、ヒットしたプリミティブ（三角形）のインデックス `hitPrimIdx` を取得します。
        *   ペイロード `p0` から、レイを発射したメッシュAの三角形インデックス `rayOriginTriIdx` を取得します。
        *   **if (元プリミティブヒット):**
            *   ヒットしたメッシュBの元の三角形インデックス `hitTriIdx = hitPrimIdx` を取得します。
            *   (自己衝突の場合): `if (rayOriginTriIdx == hitTriIdx) return;` // 自明な衝突を無視
            *   `atomicAdd(params.collisionCounter, 1)` でカウンターをインクリメントします。
            *   結果バッファに `(rayOriginTriIdx, hitTriIdx)` ペアを書き込みます（バッファオーバーフローチェックが必要な場合あり）。
        *   **else if (補助プリミティブヒット):**
            *   補助三角形インデックス `hitAuxTriIdx = hitPrimIdx` から、対応するメッシュBの*元の*三角形インデックス `hitOrigTriIdx` と、その三角形内の*元の辺*インデックス `hitEdgeIdx` を計算します。 (例: `hitOrigTriIdx = (hitAuxTriIdx - numOriginalTrisB) / 3`, `hitEdgeIdx = (hitAuxTriIdx - numOriginalTrisB) % 3`)。
            *   (自己衝突の場合): `if (rayOriginTriIdx == hitOrigTriIdx) return;`
            *   SBTデータまたはParams経由でメッシュBの頂点/インデックスにアクセスし、`hitOrigTriIdx` の頂点 `vb0, vb1, vb2` を取得します。
            *   `hitEdgeIdx` に対応する元の辺 `E'` (例: edge 0 -> vb0, vb1) を特定します。
            *   `optixGetWorldRayOrigin()`, `optixGetWorldRayDirection()`, `optixGetRayTmax()` を使って交差点 `P` を計算します: `P = origin + t * direction`。
            *   点 `P` が線分 `E'` 上にあるか判定します（許容誤差 `eps'` を考慮）。
                *   `dot(P - vb0, vb1 - vb0)` が `0` と `dot(vb1 - vb0, vb1 - vb0)` の間にあるか？
                *   `distance_squared(P, line_segment)` が `eps'^2` より小さいか？
            *   もし線分上にあれば:
                *   `atomicAdd(params.collisionCounter, 1)`。
                *   結果バッファに `(rayOriginTriIdx, hitOrigTriIdx)` ペアを書き込みます。
        *   *(Mochiは全ての衝突を見つけたいので、通常は `optixTerminateRay` や `optixIgnoreIntersection` は呼び出しません)*。

*   **`intersectKernelTriangle_impl`:**
    *   上記 `intersect_impl` と似ていますが、RayGenは指定された `inputTriangle` の3辺からのみレイを発射します。
    *   結果は、ヒットした元の三角形のインデックス（intのvector）のみです。

**3. 設計上の考慮事項**

*   **OptiX初期化:** `optixInit()` は通常、アプリケーション起動時に一度だけ呼びます。`OptixDeviceContext` はプラグインのロード/アンロード時に作成/破棄するのが適切かもしれません。Pipeline, Module, Program Groupsも同様にキャッシュ可能です。
*   **SBTデータ:** AnyHitシェーダーが必要とするデータ（バッファポインタ、ジオメトリタイプフラグなど）を効率的に渡す方法を検討します。SBTレコードに直接埋め込むか、`Params` 経由で渡します。
*   **メモリ管理:** `MochiKernelData_impl` のデストラクタで、確保した全てのGPUメモリ（頂点、インデックス、GASバッファ）を確実に解放します (`cudaFree`)。
*   **補助三角形の頂点:** 上記では既存の頂点インデックスを再利用する前提で設計しましたが、もし論文が示すようにεオフセットした新しい頂点を生成する必要がある場合、`d_vertices` バッファを拡張し、補助三角形生成カーネルで新しい頂点を書き込む必要があります。GASビルド時の `vertexBuffers` も適切に設定します。これは実装を複雑化させますが、精度が向上する可能性があります。
*   **自己衝突:** `mochiKernelIntersect_impl` 内で `selfIntersection` フラグをチェックし、AnyHitシェーダー内で `rayOriginTriIdx == hitTriIdx` のチェックを有効/無効にします。
*   **エラーハンドリング:** CUDAおよびOptiXのAPI呼び出しには必ずエラーチェックを入れます (`CUDA_CHECK`, `OPTIX_CHECK`)。
*   **パフォーマンス:**
    *   GASビルドオプション (`OPTIX_BUILD_FLAG_PREFER_FAST_TRACE` など) を調整します。
    *   カーネルのブロック/グリッドサイズを最適化します。
    *   AnyHitシェーダー内の計算量を最小限に抑えます。
    *   可能であれば非同期CUDAストリームを使用します。

この設計により、論文のアルゴリズムを効率的にGPU上で実行し、MayaのC++コードとの間でデータを適切に受け渡すことができるはずです。詳細なCUDAカーネルやOptiXシェーダーの実装は、具体的な計算式やOptiX APIの知識に基づいて行う必要があります。
 *
 *
 * */

#include "MochiKernelImpl.h"

#include "../cuda_utils/MochiKernelPODs.h"
#include "../cuda_utils/vec3_cu.hpp"
#include "../cuda_utils/point_cu.hpp"
#include "../cuda_utils/transfo.hpp"

 // #include "MochiKernelShaders_ptx.h"

#include <optix.h>
#include <optix_stubs.h>
#include <optix_device.h> // For device-side functions like optixGetPrimitiveIndex()
#include <optix_function_table_definition.h> // Needed for static linking or function table access
#include <cuda_runtime.h>

#include <fstream>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <cstring> // For memcpy
#include <cmath>   // For fabsf, sqrtf

const char MochiKernel_ptx[] = R"(
//
// Generated by NVIDIA NVVM Compiler
//
// Compiler Build ID: CL-35813241
// Cuda compilation tools, release 12.9, V12.9.41
// Based on NVVM 7.0.1
//

.version 8.8
.target sm_86
.address_size 64

	// .globl	__raygen__shootEdgeRays
.extern .func  (.param .b32 func_retval0) vprintf
(
	.param .b64 vprintf_param_0,
	.param .b64 vprintf_param_1
)
;
.const .align 8 .b8 params[64];
.global .align 1 .b8 $str[28] = {67, 111, 108, 108, 105, 115, 105, 111, 110, 32, 98, 117, 102, 102, 101, 114, 32, 111, 118, 101, 114, 102, 108, 111, 119, 33, 10};
.global .align 1 .b8 $str$1[51] = {65, 72, 32, 69, 114, 114, 111, 114, 58, 32, 72, 105, 116, 32, 65, 117, 120, 32, 83, 66, 84, 32, 98, 117, 116, 32, 80, 114, 105, 109, 73, 100, 120, 32, 37, 117, 32, 60, 32, 110, 117, 109, 79, 114, 105, 103, 32, 37, 117, 10};
.global .align 1 .b8 $str$2[55] = {65, 72, 32, 69, 114, 114, 111, 114, 58, 32, 77, 105, 115, 115, 105, 110, 103, 32, 118, 101, 114, 116, 101, 120, 47, 105, 110, 100, 101, 120, 32, 100, 97, 116, 97, 32, 102, 111, 114, 32, 97, 117, 120, 32, 104, 105, 116, 32, 99, 104, 101, 99, 107, 10};
.global .align 1 .b8 $str$3[37] = {65, 72, 32, 69, 114, 114, 111, 114, 58, 32, 85, 110, 107, 110, 111, 119, 110, 32, 83, 66, 84, 32, 103, 101, 111, 109, 101, 116, 114, 121, 32, 116, 121, 112, 101, 10};

.visible .entry __raygen__shootEdgeRays()
{
	.reg .pred 	%p<6>;
	.reg .f32 	%f<45>;
	.reg .b32 	%r<80>;
	.reg .b64 	%rd<14>;


	// begin inline asm
	call (%r3), _optix_get_launch_index_x, ();
	// end inline asm
	// begin inline asm
	call (%r4), _optix_get_launch_index_y, ();
	// end inline asm
	ld.const.u32 	%r5, [params+40];
	setp.ge.u32 	%p1, %r3, %r5;
	@%p1 bra 	$L__BB0_9;

	ld.const.u64 	%rd1, [params+32];
	cvta.to.global.u64 	%rd2, %rd1;
	mul.wide.u32 	%rd3, %r3, 12;
	add.s64 	%rd4, %rd2, %rd3;
	ld.const.u64 	%rd5, [params+24];
	cvta.to.global.u64 	%rd6, %rd5;
	ld.global.u32 	%r6, [%rd4];
	mul.wide.s32 	%rd7, %r6, 12;
	add.s64 	%rd8, %rd6, %rd7;
	ld.global.f32 	%f1, [%rd8];
	ld.global.f32 	%f2, [%rd8+4];
	ld.global.f32 	%f3, [%rd8+8];
	ld.global.u32 	%r7, [%rd4+4];
	mul.wide.s32 	%rd9, %r7, 12;
	add.s64 	%rd10, %rd6, %rd9;
	ld.global.f32 	%f4, [%rd10];
	ld.global.f32 	%f5, [%rd10+4];
	ld.global.f32 	%f6, [%rd10+8];
	ld.global.u32 	%r8, [%rd4+8];
	mul.wide.s32 	%rd11, %r8, 12;
	add.s64 	%rd12, %rd6, %rd11;
	ld.global.f32 	%f40, [%rd12];
	ld.global.f32 	%f39, [%rd12+4];
	ld.global.f32 	%f44, [%rd12+8];
	setp.eq.s32 	%p2, %r4, 0;
	@%p2 bra 	$L__BB0_6;

	setp.eq.s32 	%p3, %r4, 1;
	@%p3 bra 	$L__BB0_5;

	setp.ne.s32 	%p4, %r4, 2;
	@%p4 bra 	$L__BB0_9;

	sub.ftz.f32 	%f43, %f1, %f40;
	sub.ftz.f32 	%f42, %f2, %f39;
	sub.ftz.f32 	%f41, %f3, %f44;
	bra.uni 	$L__BB0_7;

$L__BB0_6:
	sub.ftz.f32 	%f43, %f4, %f1;
	sub.ftz.f32 	%f42, %f5, %f2;
	sub.ftz.f32 	%f41, %f6, %f3;
	mov.f32 	%f39, %f2;
	mov.f32 	%f40, %f1;
	mov.f32 	%f44, %f3;
	bra.uni 	$L__BB0_7;

$L__BB0_5:
	sub.ftz.f32 	%f43, %f40, %f4;
	sub.ftz.f32 	%f42, %f39, %f5;
	sub.ftz.f32 	%f41, %f44, %f6;
	mov.f32 	%f39, %f5;
	mov.f32 	%f40, %f4;
	mov.f32 	%f44, %f6;

$L__BB0_7:
	mul.ftz.f32 	%f26, %f43, %f43;
	fma.rn.ftz.f32 	%f27, %f42, %f42, %f26;
	fma.rn.ftz.f32 	%f28, %f41, %f41, %f27;
	sqrt.approx.ftz.f32 	%f25, %f28;
	setp.lt.ftz.f32 	%p5, %f25, 0f3727C5AC;
	@%p5 bra 	$L__BB0_9;

	rcp.approx.ftz.f32 	%f38, %f25;
	mul.ftz.f32 	%f32, %f43, %f38;
	mul.ftz.f32 	%f33, %f42, %f38;
	mul.ftz.f32 	%f34, %f41, %f38;
	ld.const.u64 	%rd13, [params];
	add.ftz.f32 	%f36, %f25, 0fB727C5AC;
	mov.f32 	%f37, 0f00000000;
	mov.u32 	%r45, 2;
	mov.u32 	%r47, 1;
	mov.u32 	%r79, 0;
	// begin inline asm
	call(%r9,%r10,%r11,%r12,%r13,%r14,%r15,%r16,%r17,%r18,%r19,%r20,%r21,%r22,%r23,%r24,%r25,%r26,%r27,%r28,%r29,%r30,%r31,%r32,%r33,%r34,%r35,%r36,%r37,%r38,%r39,%r40),_optix_trace_typed_32,(%r79,%rd13,%f40,%f39,%f44,%f32,%f33,%f34,%f37,%f36,%f37,%r47,%r79,%r79,%r45,%r79,%r47,%r3,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79,%r79);
	// end inline asm

$L__BB0_9:
	ret;

}
	// .globl	__anyhit__processHit
.visible .entry __anyhit__processHit()
{
	.local .align 8 .b8 	__local_depot1[8];
	.reg .b64 	%SP;
	.reg .b64 	%SPL;
	.reg .pred 	%p<24>;
	.reg .b16 	%rs<5>;
	.reg .f32 	%f<60>;
	.reg .b32 	%r<35>;
	.reg .b64 	%rd<47>;


	mov.u64 	%SPL, __local_depot1;
	cvta.local.u64 	%SP, %SPL;
	// begin inline asm
	call (%rd6), _optix_get_sbt_data_ptr_64, ();
	// end inline asm
	// begin inline asm
	call (%r12), _optix_read_primitive_idx, ();
	// end inline asm
	mov.u32 	%r14, 0;
	// begin inline asm
	call (%r13), _optix_get_payload, (%r14);
	// end inline asm
	ld.u32 	%r15, [%rd6+32];
	setp.eq.s32 	%p1, %r15, 0;
	@%p1 bra 	$L__BB1_22;

	setp.ne.s32 	%p2, %r15, 1;
	@%p2 bra 	$L__BB1_28;

	ld.u32 	%r3, [%rd6+64];
	setp.lt.u32 	%p3, %r12, %r3;
	@%p3 bra 	$L__BB1_21;
	bra.uni 	$L__BB1_3;

$L__BB1_21:
	add.u64 	%rd31, %SP, 0;
	add.u64 	%rd32, %SPL, 0;
	st.local.v2.u32 	[%rd32], {%r12, %r3};
	mov.u64 	%rd33, $str$1;
	cvta.global.u64 	%rd34, %rd33;
	{ // callseq 2, 0
	.reg .b32 temp_param_reg;
	.param .b64 param0;
	st.param.b64 	[param0+0], %rd34;
	.param .b64 param1;
	st.param.b64 	[param1+0], %rd31;
	.param .b32 retval0;
	call.uni (retval0), 
	vprintf, 
	(
	param0, 
	param1
	);
	ld.param.b32 	%r27, [retval0+0];
	} // callseq 2
	bra.uni 	$L__BB1_29;

$L__BB1_22:
	ld.u64 	%rd5, [%rd6+56];
	setp.eq.s64 	%p19, %rd5, 0;
	mov.u32 	%r34, -1;
	@%p19 bra 	$L__BB1_24;

	mul.wide.u32 	%rd35, %r12, 4;
	add.s64 	%rd36, %rd5, %rd35;
	ld.u32 	%r34, [%rd36];

$L__BB1_24:
	ld.const.u8 	%rs4, [params+56];
	setp.ne.s16 	%p20, %rs4, 0;
	setp.eq.s32 	%p21, %r13, %r12;
	and.pred  	%p22, %p21, %p20;
	@%p22 bra 	$L__BB1_29;

	ld.const.u64 	%rd37, [params+16];
	cvta.to.global.u64 	%rd38, %rd37;
	atom.global.add.u32 	%r11, [%rd38], 1;
	ld.const.u32 	%r29, [params+60];
	setp.lt.u32 	%p23, %r11, %r29;
	@%p23 bra 	$L__BB1_27;
	bra.uni 	$L__BB1_26;

$L__BB1_27:
	ld.const.u64 	%rd41, [params+8];
	cvta.to.global.u64 	%rd42, %rd41;
	mul.wide.u32 	%rd43, %r11, 16;
	add.s64 	%rd44, %rd42, %rd43;
	st.global.u32 	[%rd44], %r13;
	mov.u32 	%r31, -1;
	st.global.u32 	[%rd44+8], %r31;
	st.global.u32 	[%rd44+4], %r12;
	st.global.u32 	[%rd44+12], %r34;
	bra.uni 	$L__BB1_29;

$L__BB1_28:
	mov.u64 	%rd45, $str$3;
	cvta.global.u64 	%rd46, %rd45;
	{ // callseq 4, 0
	.reg .b32 temp_param_reg;
	.param .b64 param0;
	st.param.b64 	[param0+0], %rd46;
	.param .b64 param1;
	st.param.b64 	[param1+0], 0;
	.param .b32 retval0;
	call.uni (retval0), 
	vprintf, 
	(
	param0, 
	param1
	);
	ld.param.b32 	%r32, [retval0+0];
	} // callseq 4

$L__BB1_29:
	ret;

$L__BB1_3:
	sub.s32 	%r4, %r12, %r3;
	mul.wide.u32 	%rd7, %r4, -1431655765;
	shr.u64 	%rd8, %rd7, 33;
	cvt.u32.u64 	%r5, %rd8;
	ld.u64 	%rd2, [%rd6+56];
	setp.eq.s64 	%p4, %rd2, 0;
	mov.u32 	%r33, -1;
	@%p4 bra 	$L__BB1_5;

	mul.wide.u32 	%rd9, %r5, 4;
	add.s64 	%rd10, %rd2, %rd9;
	ld.u32 	%r33, [%rd10];

$L__BB1_5:
	ld.const.u8 	%rs1, [params+56];
	setp.ne.s16 	%p5, %rs1, 0;
	setp.eq.s32 	%p6, %r13, %r5;
	and.pred  	%p7, %p6, %p5;
	@%p7 bra 	$L__BB1_29;

	ld.u64 	%rd3, [%rd6+48];
	setp.eq.s64 	%p8, %rd3, 0;
	@%p8 bra 	$L__BB1_20;

	ld.u64 	%rd4, [%rd6+40];
	setp.eq.s64 	%p9, %rd4, 0;
	@%p9 bra 	$L__BB1_20;

	mul.wide.u32 	%rd11, %r5, 12;
	add.s64 	%rd12, %rd3, %rd11;
	ld.u32 	%r17, [%rd12];
	mul.wide.s32 	%rd13, %r17, 12;
	add.s64 	%rd14, %rd4, %rd13;
	ld.f32 	%f56, [%rd14];
	ld.f32 	%f57, [%rd14+4];
	ld.f32 	%f58, [%rd14+8];
	ld.u32 	%r18, [%rd12+4];
	mul.wide.s32 	%rd15, %r18, 12;
	add.s64 	%rd16, %rd4, %rd15;
	ld.f32 	%f4, [%rd16];
	ld.f32 	%f5, [%rd16+4];
	ld.f32 	%f6, [%rd16+8];
	ld.u32 	%r19, [%rd12+8];
	mul.wide.s32 	%rd17, %r19, 12;
	add.s64 	%rd18, %rd4, %rd17;
	ld.f32 	%f7, [%rd18];
	ld.f32 	%f8, [%rd18+4];
	ld.f32 	%f9, [%rd18+8];
	mul.lo.s32 	%r21, %r5, 3;
	sub.s32 	%r22, %r4, %r21;
	cvt.u16.u32 	%rs2, %r22;
	and.b16  	%rs3, %rs2, 255;
	setp.eq.s16 	%p10, %rs3, 0;
	mov.f32 	%f53, %f4;
	mov.f32 	%f54, %f5;
	mov.f32 	%f55, %f6;
	@%p10 bra 	$L__BB1_12;

	setp.eq.s16 	%p11, %rs2, 1;
	@%p11 bra 	$L__BB1_11;

	setp.ne.s16 	%p12, %rs2, 2;
	mov.f32 	%f53, %f56;
	mov.f32 	%f54, %f57;
	mov.f32 	%f55, %f58;
	mov.f32 	%f56, %f7;
	mov.f32 	%f57, %f8;
	mov.f32 	%f58, %f9;
	@%p12 bra 	$L__BB1_29;
	bra.uni 	$L__BB1_12;

$L__BB1_26:
	mov.u64 	%rd39, $str;
	cvta.global.u64 	%rd40, %rd39;
	{ // callseq 3, 0
	.reg .b32 temp_param_reg;
	.param .b64 param0;
	st.param.b64 	[param0+0], %rd40;
	.param .b64 param1;
	st.param.b64 	[param1+0], 0;
	.param .b32 retval0;
	call.uni (retval0), 
	vprintf, 
	(
	param0, 
	param1
	);
	ld.param.b32 	%r30, [retval0+0];
	} // callseq 3
	bra.uni 	$L__BB1_29;

$L__BB1_20:
	mov.u64 	%rd29, $str$2;
	cvta.global.u64 	%rd30, %rd29;
	{ // callseq 1, 0
	.reg .b32 temp_param_reg;
	.param .b64 param0;
	st.param.b64 	[param0+0], %rd30;
	.param .b64 param1;
	st.param.b64 	[param1+0], 0;
	.param .b32 retval0;
	call.uni (retval0), 
	vprintf, 
	(
	param0, 
	param1
	);
	ld.param.b32 	%r26, [retval0+0];
	} // callseq 1
	bra.uni 	$L__BB1_29;

$L__BB1_11:
	mov.f32 	%f53, %f7;
	mov.f32 	%f54, %f8;
	mov.f32 	%f55, %f9;
	mov.f32 	%f56, %f4;
	mov.f32 	%f57, %f5;
	mov.f32 	%f58, %f6;

$L__BB1_12:
	// begin inline asm
	call (%f27), _optix_get_ray_tmax, ();
	// end inline asm
	// begin inline asm
	call (%f28), _optix_get_world_ray_origin_x, ();
	// end inline asm
	// begin inline asm
	call (%f29), _optix_get_world_ray_origin_y, ();
	// end inline asm
	// begin inline asm
	call (%f30), _optix_get_world_ray_origin_z, ();
	// end inline asm
	// begin inline asm
	call (%f31), _optix_get_world_ray_direction_x, ();
	// end inline asm
	// begin inline asm
	call (%f32), _optix_get_world_ray_direction_y, ();
	// end inline asm
	// begin inline asm
	call (%f33), _optix_get_world_ray_direction_z, ();
	// end inline asm
	fma.rn.ftz.f32 	%f34, %f27, %f31, %f28;
	fma.rn.ftz.f32 	%f35, %f27, %f32, %f29;
	fma.rn.ftz.f32 	%f36, %f27, %f33, %f30;
	sub.ftz.f32 	%f16, %f34, %f56;
	sub.ftz.f32 	%f17, %f35, %f57;
	sub.ftz.f32 	%f18, %f36, %f58;
	sub.ftz.f32 	%f19, %f53, %f56;
	sub.ftz.f32 	%f20, %f54, %f57;
	mul.ftz.f32 	%f37, %f20, %f20;
	fma.rn.ftz.f32 	%f38, %f19, %f19, %f37;
	sub.ftz.f32 	%f21, %f55, %f58;
	fma.rn.ftz.f32 	%f22, %f21, %f21, %f38;
	setp.lt.ftz.f32 	%p13, %f22, 0f2EDBE6FE;
	@%p13 bra 	$L__BB1_15;
	bra.uni 	$L__BB1_13;

$L__BB1_15:
	mul.ftz.f32 	%f51, %f17, %f17;
	fma.rn.ftz.f32 	%f52, %f16, %f16, %f51;
	fma.rn.ftz.f32 	%f59, %f18, %f18, %f52;
	bra.uni 	$L__BB1_16;

$L__BB1_13:
	mul.ftz.f32 	%f39, %f20, %f17;
	fma.rn.ftz.f32 	%f40, %f19, %f16, %f39;
	fma.rn.ftz.f32 	%f23, %f21, %f18, %f40;
	setp.lt.ftz.f32 	%p14, %f23, 0fB727C5AC;
	add.ftz.f32 	%f41, %f22, 0f3727C5AC;
	setp.gt.ftz.f32 	%p15, %f23, %f41;
	or.pred  	%p16, %p14, %p15;
	@%p16 bra 	$L__BB1_29;

	div.approx.ftz.f32 	%f42, %f23, %f22;
	mul.ftz.f32 	%f43, %f19, %f42;
	mul.ftz.f32 	%f44, %f20, %f42;
	mul.ftz.f32 	%f45, %f21, %f42;
	sub.ftz.f32 	%f46, %f16, %f43;
	sub.ftz.f32 	%f47, %f17, %f44;
	sub.ftz.f32 	%f48, %f18, %f45;
	mul.ftz.f32 	%f49, %f47, %f47;
	fma.rn.ftz.f32 	%f50, %f46, %f46, %f49;
	fma.rn.ftz.f32 	%f59, %f48, %f48, %f50;

$L__BB1_16:
	setp.geu.ftz.f32 	%p17, %f59, 0f2EDBE6FE;
	@%p17 bra 	$L__BB1_29;

	ld.const.u64 	%rd21, [params+16];
	cvta.to.global.u64 	%rd22, %rd21;
	atom.global.add.u32 	%r8, [%rd22], 1;
	ld.const.u32 	%r23, [params+60];
	setp.lt.u32 	%p18, %r8, %r23;
	@%p18 bra 	$L__BB1_19;
	bra.uni 	$L__BB1_18;

$L__BB1_19:
	ld.const.u64 	%rd25, [params+8];
	cvta.to.global.u64 	%rd26, %rd25;
	mul.wide.u32 	%rd27, %r8, 16;
	add.s64 	%rd28, %rd26, %rd27;
	st.global.u32 	[%rd28], %r13;
	mov.u32 	%r25, -1;
	st.global.u32 	[%rd28+8], %r25;
	st.global.u32 	[%rd28+4], %r5;
	st.global.u32 	[%rd28+12], %r33;
	bra.uni 	$L__BB1_29;

$L__BB1_18:
	mov.u64 	%rd23, $str;
	cvta.global.u64 	%rd24, %rd23;
	{ // callseq 0, 0
	.reg .b32 temp_param_reg;
	.param .b64 param0;
	st.param.b64 	[param0+0], %rd24;
	.param .b64 param1;
	st.param.b64 	[param1+0], 0;
	.param .b32 retval0;
	call.uni (retval0), 
	vprintf, 
	(
	param0, 
	param1
	);
	ld.param.b32 	%r24, [retval0+0];
	} // callseq 0
	bra.uni 	$L__BB1_29;

}
	// .globl	__miss__miss
.visible .entry __miss__miss()
{



	ret;

}


)";

// --- CUDA/OptiX Helper Macros ---
#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t error = call;                                           \
        if (error != cudaSuccess) {                                         \
            fprintf(stderr, "CUDA Error (%s:%d): %s\n", __FILE__, __LINE__, cudaGetErrorString(error)); \
            throw std::runtime_error("CUDA error");                         \
        }                                                                   \
    } while (0)

#define OPTIX_CHECK(call)                                                   \
    do {                                                                    \
        OptixResult res = call;                                             \
        if (res != OPTIX_SUCCESS) {                                         \
            fprintf(stderr, "OptiX Error (%s:%d): %s\n", __FILE__, __LINE__, optixGetErrorName(res)); \
            throw std::runtime_error(std::string("OptiX error: ") + optixGetErrorName(res)); \
        }                                                                   \
    } while (0)

#define INTERSECTION_EPSILON 1e-5f // Tolerance for point-on-segment check


// --- Forward Declarations for OptiX Programs ---
// These need to be defined in separate PTX files or using NVRTC at runtime
// For simplicity, we'll assume they exist and are linked.
struct Params {
    OptixTraversableHandle targetGas;             // BVH of the mesh being hit
    MochiCollisionPairPOD* collisionBuffer;       // Output buffer for collision pairs
    unsigned int*          collisionCounter;      // Atomic counter for number of collisions
    Point_cu*              meshAVertices;         // Vertex buffer of the mesh shooting rays
    int3*                  meshAIndices;          // Original triangle indices of the mesh shooting rays
    unsigned int           meshANumOriginalTriangles; // Number of original triangles in the mesh shooting rays
    int*                   meshAOriginalFaceIndices; // Optional: Face indices for mesh A
    bool                   selfIntersectionCheck;   // Flag to enable self-hit checks
    unsigned int           maxCollisions;           // Size of collisionBuffer
};

extern "C" __constant__ Params params; // Launch parameters accessible globally
extern "C" __global__ void __raygen__shootEdgeRays();
extern "C" __global__ void __anyhit__processHit();
extern "C" __global__ void __miss__miss();

// --- Global/Static OptiX State (Simplified Example) ---
// A real implementation would manage this more robustly, potentially
// sharing the context across plugin instances or managing it per Maya session.
static OptixDeviceContext g_optixContext = nullptr;
static OptixPipeline      g_optixPipeline = nullptr;
static OptixModule        g_optixModule = nullptr;
static OptixProgramGroup  g_raygenPG = nullptr;
static OptixProgramGroup  g_missPG = nullptr;
static OptixProgramGroup  g_hitgroupPG = nullptr; // Assuming one hitgroup for simplicity here
static OptixShaderBindingTable g_sbt = {};
static CUdeviceptr        g_d_sbt_raygen_record = 0;
static CUdeviceptr        g_d_sbt_miss_record = 0;
static CUdeviceptr        g_d_sbt_hitgroup_records = 0; // Array for original and aux

// ... other functions ...
struct RayGenData {
    alignas(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    // Add any user-specific raygen data here if needed
};

struct MissData {
    alignas(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    // Add any user-specific miss data here if needed
};

struct HitGroupData {
    alignas(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    enum GeoType { ORIGINAL, AUXILIARY };
    GeoType type;
    Point_cu* vertices;
    int3*     indices; // Original or Aux indices depending on type
    int*      originalFaceIndices; // Only valid for ORIGINAL type
    unsigned int numOriginalTriangles; // Needed by Aux hit to map back
};

// --- Internal Data Structure Definition ---
struct MochiKernelData_impl {
    OptixTraversableHandle gasHandle = 0;
    CUdeviceptr d_gas_output_buffer = 0;

    CUdeviceptr d_vertices = 0;
    CUdeviceptr d_original_indices = 0;
    CUdeviceptr d_aux_indices = 0;
    CUdeviceptr d_original_face_indices = 0;

    unsigned int numVertices = 0;
    unsigned int numOriginalTriangles = 0;
    unsigned int numAuxTriangles = 0;
    unsigned int numTotalPrimitives = 0;
};

// ... other global handles (pipeline, module, etc.) ...
static void context_log_cb(unsigned int level, const char* tag, const char* message, void* /*cbdata */)
{
    if (level <= 4) { // Adjust log level as needed
        std::cout << "[" << tag << "] " << message << std::endl;
    }
}

// This should be called from initializePlugin
extern "C" bool initializeMochiOptiX()
{
    if (g_optixContext) {
        std::cout << "OptiX already initialized for MochiKernel." << std::endl;
        return true;
    }

    std::cout << "Initializing OptiX for MochiKernel..." << std::endl;
    char log[2048];
    size_t sizeof_log = sizeof(log);

    try {
        // 1. Initialize CUDA & OptiX context
        CUDA_CHECK(cudaFree(0));
        OPTIX_CHECK(optixInit());
        OptixDeviceContextOptions options = {};
        options.logCallbackFunction = &context_log_cb;
        options.logCallbackLevel = 4;
        CUcontext cuCtx = 0;
        OPTIX_CHECK(optixDeviceContextCreate(cuCtx, &options, &g_optixContext));
        std::cout << "OptiX context created." << std::endl;

        // 2. Define Compile Options
        OptixPipelineCompileOptions pipelineCompileOptions = {}; // Define this early
        pipelineCompileOptions.usesMotionBlur = 0;
        pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
        pipelineCompileOptions.numPayloadValues = 1;
        pipelineCompileOptions.numAttributeValues = 2;
        pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE; // Use DEBUG for debugging
        pipelineCompileOptions.pipelineLaunchParamsVariableName = "params";

        OptixModuleCompileOptions module_compile_options = {};
        module_compile_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
        module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
        #if defined(DEBUG) || defined(_DEBUG)
        module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MODERATE;
        #else
        module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
        #endif

        // 3. Create Module from Embedded PTX String
        sizeof_log = sizeof(log);
        std::cout << "Creating module from embedded PTX..." << std::endl;
        OPTIX_CHECK(optixModuleCreate(g_optixContext,
                                      &module_compile_options,
                                      &pipelineCompileOptions,
                                      MochiKernel_ptx, // Use the variable from MochiKernel_ptx.h
                                      strlen(MochiKernel_ptx), // Use strlen for char array
                                      log, &sizeof_log,
                                      &g_optixModule));
        if (sizeof_log > 1) { std::cout << "OptiX Module Log: " << log << std::endl; }
        std::cout << "OptiX Module created." << std::endl;

        // 4. Create Program Groups (using function names from the PTX)
        OptixProgramGroupOptions pgOptions = {};
        OptixProgramGroupDesc    programGroupDescs[3];

        // Raygen
        memset(&programGroupDescs[0], 0, sizeof(OptixProgramGroupDesc));
        programGroupDescs[0].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
        programGroupDescs[0].raygen.module = g_optixModule;
        programGroupDescs[0].raygen.entryFunctionName = "__raygen__shootEdgeRays"; // Must match name in PTX
        sizeof_log = sizeof(log);
        OPTIX_CHECK(optixProgramGroupCreate(g_optixContext, &programGroupDescs[0], 1, &pgOptions, log, &sizeof_log, &g_raygenPG));
        if (sizeof_log > 1) { std::cout << "Raygen PG Log: " << log << std::endl; }

        // Miss
        memset(&programGroupDescs[1], 0, sizeof(OptixProgramGroupDesc));
        programGroupDescs[1].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
        programGroupDescs[1].miss.module = g_optixModule;
        programGroupDescs[1].miss.entryFunctionName = "__miss__miss"; // Must match name in PTX
        sizeof_log = sizeof(log);
        OPTIX_CHECK(optixProgramGroupCreate(g_optixContext, &programGroupDescs[1], 1, &pgOptions, log, &sizeof_log, &g_missPG));
        if (sizeof_log > 1) { std::cout << "Miss PG Log: " << log << std::endl; }

        // Hitgroup
        memset(&programGroupDescs[2], 0, sizeof(OptixProgramGroupDesc));
        programGroupDescs[2].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
        programGroupDescs[2].hitgroup.moduleAH = g_optixModule;
        programGroupDescs[2].hitgroup.entryFunctionNameAH = "__anyhit__processHit"; // Must match name in PTX
        sizeof_log = sizeof(log);
        OPTIX_CHECK(optixProgramGroupCreate(g_optixContext, &programGroupDescs[2], 1, &pgOptions, log, &sizeof_log, &g_hitgroupPG));
        if (sizeof_log > 1) { std::cout << "Hitgroup PG Log: " << log << std::endl; }
        std::cout << "Program Groups created." << std::endl;

        // 5. Create Pipeline
        OptixPipelineLinkOptions pipelineLinkOptions = {};
        pipelineLinkOptions.maxTraceDepth = 1;
        OptixProgramGroup programGroups[] = { g_raygenPG, g_missPG, g_hitgroupPG };
        sizeof_log = sizeof(log);
        OPTIX_CHECK(optixPipelineCreate(g_optixContext,
                                        &pipelineCompileOptions,
                                        &pipelineLinkOptions,
                                        programGroups,
                                        sizeof(programGroups) / sizeof(programGroups[0]),
                                        log, &sizeof_log,
                                        &g_optixPipeline));
        if (sizeof_log > 1) { std::cout << "Pipeline Log: " << log << std::endl; }
        std::cout << "Pipeline created." << std::endl;

        // 6. Create Shader Binding Table (SBT)
        //    (SBT creation logic remains the same as previous version)
        // ... (Allocate g_d_sbt_* records, pack headers, copy data) ...
        // --- Create Shader Binding Table (SBT) ---
        std::cout << "Creating SBT..." << std::endl;

        // RayGen Record
        RayGenData rgSBT; // Host-side SBT record
        OPTIX_CHECK(optixSbtRecordPackHeader(g_raygenPG, &rgSBT));
        // No user data for RayGenData in this example, or set it here: rgSBT.data.some_field = value;
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&g_d_sbt_raygen_record), sizeof(RayGenData)));
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(g_d_sbt_raygen_record), &rgSBT, sizeof(RayGenData), cudaMemcpyHostToDevice));

        // Miss Record
        MissData msSBT; // Host-side SBT record
        OPTIX_CHECK(optixSbtRecordPackHeader(g_missPG, &msSBT));
        // No user data for MissData in this example
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&g_d_sbt_miss_record), sizeof(MissData)));
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(g_d_sbt_miss_record), &msSBT, sizeof(MissData), cudaMemcpyHostToDevice));

        // HitGroup Records (Original + Auxiliary as per original code)
        HitGroupData hgSBTs[2]; // Host-side SBT records

        // Record 0: Original
        hgSBTs[0].type = HitGroupData::ORIGINAL;
        // Other data for hgSBTs[0] can be set here
        OPTIX_CHECK(optixSbtRecordPackHeader(g_hitgroupPG, &hgSBTs[0]));

        // Record 1: Auxiliary
        hgSBTs[1].type = HitGroupData::AUXILIARY;
        // Other data for hgSBTs[1] can be set here
        OPTIX_CHECK(optixSbtRecordPackHeader(g_hitgroupPG, &hgSBTs[1]));

        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&g_d_sbt_hitgroup_records), sizeof(HitGroupData) * 2));
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(g_d_sbt_hitgroup_records), hgSBTs, sizeof(HitGroupData) * 2, cudaMemcpyHostToDevice));

        // Setup the OptixShaderBindingTable structure
        g_sbt.raygenRecord = g_d_sbt_raygen_record;
        g_sbt.missRecordBase = g_d_sbt_miss_record;
        g_sbt.missRecordStrideInBytes = sizeof(MissData);
        g_sbt.missRecordCount = 1;
        g_sbt.hitgroupRecordBase = g_d_sbt_hitgroup_records;
        g_sbt.hitgroupRecordStrideInBytes = sizeof(HitGroupData);
        g_sbt.hitgroupRecordCount = 2; // One for original, one for auxiliary

        std::cout << "SBT created and uploaded." << std::endl;


    } catch (const std::exception& e) {
         std::cerr << "OptiX Initialization failed: " << e.what() << std::endl;
         cleanupMochiOptiX(); // Use the existing cleanup function
         return false;
    }

    std::cout << "OptiX for MochiKernel initialized successfully." << std::endl;
    return true;
}

// This should be called from uninitializePlugin
extern "C" void cleanupMochiOptiX()
{
    std::cout << "Cleaning up OptiX for MochiKernel..." << std::endl;

    // Order of destruction should generally be reverse of creation
    if (g_d_sbt_raygen_record) cudaFree(reinterpret_cast<void*>(g_d_sbt_raygen_record));
    if (g_d_sbt_miss_record) cudaFree(reinterpret_cast<void*>(g_d_sbt_miss_record));
    if (g_d_sbt_hitgroup_records) cudaFree(reinterpret_cast<void*>(g_d_sbt_hitgroup_records));

    g_d_sbt_raygen_record = 0;
    g_d_sbt_miss_record = 0;
    g_d_sbt_hitgroup_records = 0;
    memset(&g_sbt, 0, sizeof(OptixShaderBindingTable));

    if (g_optixPipeline){ optixPipelineDestroy(g_optixPipeline); }
    g_optixPipeline = nullptr;

    if (g_hitgroupPG){ optixProgramGroupDestroy(g_hitgroupPG); }
    if (g_missPG){ optixProgramGroupDestroy(g_missPG); }
    if (g_raygenPG){ optixProgramGroupDestroy(g_raygenPG); }
    g_hitgroupPG = nullptr;
    g_missPG = nullptr;
    g_raygenPG = nullptr;

    if (g_optixModule) optixModuleDestroy(g_optixModule);
    g_optixModule = nullptr;

    if (g_optixContext) optixDeviceContextDestroy(g_optixContext);
    g_optixContext = nullptr;

    std::cout << "OptiX cleanup complete." << std::endl;
}

// --- Bridging Function Implementations ---

extern "C" MochiKernelData_impl* mochiKernelBuild_impl(
    const std::vector<Point_cu>& vertices,
    const std::vector<int>& indices,
    const std::vector<int>& faceIndices)
{

     if (!g_optixContext) {
         std::cerr << "OptiX context not initialized. Call initializeMochiOptiX() first." << std::endl;
         return nullptr;
     }
     // ... rest of the build implementation remains the same as before ...
     // (Allocate buffers, copy data, call generateAux kernel (placeholder), build GAS)
    MochiKernelData_impl* data = new MochiKernelData_impl();
    CUdeviceptr d_temp_buffer_gas = 0;
    CUdeviceptr d_sbt_offsets = 0;

    try {
        data->numVertices = vertices.size();
        data->numOriginalTriangles = indices.size() / 3;
        data->numAuxTriangles = data->numOriginalTriangles * 3;
        data->numTotalPrimitives = data->numOriginalTriangles + data->numAuxTriangles;

        if (data->numVertices == 0 || data->numOriginalTriangles == 0) {
             std::cout << "  Build warning: No vertices or triangles to build." << std::endl;
             delete data;
             return nullptr;
        }

        // 1. Allocate GPU memory (Casting to void**)
        size_t vertices_size = data->numVertices * sizeof(Point_cu);
        size_t indices_size = data->numOriginalTriangles * 3 * sizeof(int);
        size_t aux_indices_size = data->numAuxTriangles * 3 * sizeof(int);
        size_t face_indices_size = data->numOriginalTriangles * sizeof(int);

        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data->d_vertices), vertices_size));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data->d_original_indices), indices_size));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data->d_aux_indices), aux_indices_size));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data->d_original_face_indices), face_indices_size));

        // 2. Copy data to GPU
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(data->d_vertices), vertices.data(), vertices_size, cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(data->d_original_indices), indices.data(), indices_size, cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(data->d_original_face_indices), faceIndices.data(), face_indices_size, cudaMemcpyHostToDevice));

        // 3. Generate Auxiliary Triangles
        if (data->numOriginalTriangles > 0) {
            // ... (Kernel launch logic or placeholder) ...
            // Placeholder: Fill aux indices with zeros (Casting to void*)
            CUDA_CHECK(cudaMemset(reinterpret_cast<void*>(data->d_aux_indices), 0, aux_indices_size));
            // std::cout << "  Auxiliary triangle generation placeholder (zeroed memory)." << std::endl;
        }

        // 4. Build OptiX Acceleration Structure (GAS)
        OptixAccelBuildOptions accel_options = {};
        accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
        accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

        OptixBuildInput build_inputs[2] = {};
        unsigned int input_flags[1] = { OPTIX_GEOMETRY_FLAG_NONE }; // Can be defined once

        // Input 0: Original Triangles
        build_inputs[0].type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
        auto& triArray0 = build_inputs[0].triangleArray; // Use reference for convenience
        triArray0.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
        triArray0.vertexStrideInBytes = sizeof(Point_cu);
        triArray0.numVertices = data->numVertices;
        triArray0.vertexBuffers = &data->d_vertices;
        triArray0.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        triArray0.indexStrideInBytes = sizeof(int) * 3;
        triArray0.numIndexTriplets = data->numOriginalTriangles;
        triArray0.indexBuffer = data->d_original_indices;
        triArray0.flags = input_flags;
        triArray0.numSbtRecords = 1;
        triArray0.sbtIndexOffsetBuffer = 0;
        triArray0.sbtIndexOffsetSizeInBytes = 0;
        triArray0.sbtIndexOffsetStrideInBytes = 0;
        triArray0.primitiveIndexOffset = 0;

        // Input 1: Auxiliary Triangles
        build_inputs[1].type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
        auto& triArray1 = build_inputs[1].triangleArray; // Use reference
        triArray1.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
        triArray1.vertexStrideInBytes = sizeof(Point_cu);
        triArray1.numVertices = data->numVertices;
        triArray1.vertexBuffers = &data->d_vertices;
        triArray1.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        triArray1.indexStrideInBytes = sizeof(int) * 3;
        triArray1.numIndexTriplets = data->numAuxTriangles;
        triArray1.indexBuffer = data->d_aux_indices;
        triArray1.flags = input_flags;
        triArray1.numSbtRecords = 1;

        // Assign SBT offset 1 using a temporary buffer
        std::vector<unsigned int> sbt_offsets(data->numAuxTriangles, 1);
        // d_sbt_offsets = 0; // Already declared above
        if (data->numAuxTriangles > 0) {
            CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_sbt_offsets), data->numAuxTriangles * sizeof(unsigned int)));
            CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_sbt_offsets), sbt_offsets.data(), data->numAuxTriangles * sizeof(unsigned int), cudaMemcpyHostToDevice));
            triArray1.sbtIndexOffsetBuffer = d_sbt_offsets;
            triArray1.sbtIndexOffsetSizeInBytes = sizeof(unsigned int);
            triArray1.sbtIndexOffsetStrideInBytes = sizeof(unsigned int);
        } else {
            triArray1.sbtIndexOffsetBuffer = 0;
            triArray1.sbtIndexOffsetSizeInBytes = 0;
            triArray1.sbtIndexOffsetStrideInBytes = 0;
        }
        triArray1.primitiveIndexOffset = data->numOriginalTriangles;

        OptixAccelBufferSizes gas_buffer_sizes;
        OPTIX_CHECK(optixAccelComputeMemoryUsage(g_optixContext, &accel_options, build_inputs, 2, &gas_buffer_sizes));
        // std::cout << "  GAS Memory Usage: Temp=" << gas_buffer_sizes.tempSizeInBytes << ", Output=" << gas_buffer_sizes.outputSizeInBytes << std::endl;

        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_temp_buffer_gas), gas_buffer_sizes.tempSizeInBytes));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data->d_gas_output_buffer), gas_buffer_sizes.outputSizeInBytes));

        OPTIX_CHECK(optixAccelBuild(
            g_optixContext,
            0, // cudaStream
            &accel_options,
            build_inputs,
            2, // num build inputs
            d_temp_buffer_gas, gas_buffer_sizes.tempSizeInBytes,
            data->d_gas_output_buffer, gas_buffer_sizes.outputSizeInBytes,
            &data->gasHandle,
            nullptr, 0 // emitted properties
        ));

        // --- COMPACTION (Optional but recommended) ---
        // OptixAccelEmitDesc emitProperty = {};
        // emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        // emitProperty.result = (CUdeviceptr)((char*)data->d_gas_output_buffer + gas_buffer_sizes.outputSizeInBytes); // Place result after main buffer
        // size_t compactedSizeOffset = gas_buffer_sizes.outputSizeInBytes; // Store offset
        // gas_buffer_sizes.outputSizeInBytes += sizeof(uint64_t); // Add space for the size
        // 
        // OPTIX_CHECK( optixAccelBuild( g_optixContext, 0, &accel_options,
        //                             build_inputs, 2, d_temp_buffer_gas, gas_buffer_sizes.tempSizeInBytes,
        //                             data->d_gas_output_buffer, gas_buffer_sizes.outputSizeInBytes, &data->gasHandle,
        //                             &emitProperty, 1 ) );
        // 
        // uint64_t compacted_gas_size;
        // CUDA_CHECK( cudaMemcpy( &compacted_gas_size, (void*)emitProperty.result, sizeof(uint64_t), cudaMemcpyDeviceToHost ) );
        // 
        // if( compacted_gas_size < gas_buffer_sizes.outputSizeInBytes ) {
        //     std::cout << "  Applying compaction: " << gas_buffer_sizes.outputSizeInBytes << " -> " << compacted_gas_size << std::endl;
        //     CUdeviceptr d_compacted_gas_buffer;
        //     CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_compacted_gas_buffer ), compacted_gas_size ) );
        //     OPTIX_CHECK( optixAccelCompact( g_optixContext, 0, data->gasHandle, d_compacted_gas_buffer, compacted_gas_size, &data->gasHandle ) );
        //     CUDA_CHECK( cudaFree( (void*)data->d_gas_output_buffer ) ); // Free original
        //     data->d_gas_output_buffer = d_compacted_gas_buffer; // Keep compacted one
        // } else {
        //     std::cout << "  Compaction didn't reduce size." << std::endl;
        // }
        // --- END COMPACTION ---

        CUDA_CHECK(cudaFree(reinterpret_cast<void*>(d_temp_buffer_gas)));
        if (d_sbt_offsets) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(d_sbt_offsets)));
        // std::cout << "  OptiX GAS Build successful." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during MochiKernel build: " << e.what() << std::endl;
        if (data) {
             mochiKernelDestroy_impl(data);
             data = nullptr;
        }
        if (d_temp_buffer_gas) cudaFree(reinterpret_cast<void*>(d_temp_buffer_gas));
        if (d_sbt_offsets) cudaFree(reinterpret_cast<void*>(d_sbt_offsets));
        return nullptr;
    }

    return data;
}

// --- CUDA Kernel for Auxiliary Triangle Generation ---
__global__ void generateAuxTrianglesKernel(
    const Point_cu* vertices,
    const int3* originalIndices,
    int numOriginalTriangles,
    int3* auxIndices,
    float epsilon) // Small offset value
{
    int triIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (triIdx >= numOriginalTriangles) return;

    int3 vIdx = originalIndices[triIdx];
    Point_cu v0 = vertices[vIdx.x];
    Point_cu v1 = vertices[vIdx.y];
    Point_cu v2 = vertices[vIdx.z];

    // Calculate face normal
    Vec3_cu N = (v1 - v0).cross(v2 - v0);
    N.safe_normalize();
    N = N * epsilon; // Scale normal by epsilon for offset

    // Edge 0 (v0 -> v1) - Aux Triangle 0 (v0, v1+N, v1-N conceptually, indices remain same for now)
    // Edge 1 (v1 -> v2) - Aux Triangle 1 (v1, v2+N, v2-N conceptually, indices remain same for now)
    // Edge 2 (v2 -> v0) - Aux Triangle 2 (v2, v0+N, v0-N conceptually, indices remain same for now)
    //
    // IMPORTANT: Mochi paper's Figure 3 shows auxiliary triangles
    // T(VA, VB + eps*N, VB - eps*N) for edge VA-VB. This *requires* generating
    // new vertices offset by the normal. This complicates things significantly.
    //
    // ALTERNATIVE (Simpler, maybe less robust): Keep original indices for now,
    // and handle the check purely based on hitting the aux triangle's SBT record.
    // Let's proceed with the simpler approach first. If it fails, we revisit
    // generating offset vertices. The key is distinguishing the hit type.
    // We store the original triangle's indices for *all* auxiliary triangles
    // derived from it. The hit shader must know which edge it corresponds to.

    int auxBaseIdx = triIdx * 3 * 3; // 3 aux triangles per original, 3 indices per triangle

    // Aux for Edge 0 (v0-v1) -> Uses original vertices v0, v1, v1 (degenerate, or maybe v0,v1,v0?)
    // Let's try to use the original triangle's indices and rely on SBT offset + prim ID.
    // This seems insufficient. We *must* somehow link the aux hit back to the original edge.
    //
    // REVISED APPROACH: Store original edge index within the aux triangle indices?
    // This breaks the int3 format.
    //
    // FINAL REVISED APPROACH: The AnyHit shader will receive the primitive index.
    // If primIdx >= numOriginalTris, it's auxiliary.
    // auxPrimIdx = primIdx - numOriginalTris
    // origTriIdx = auxPrimIdx / 3
    // edgeInOrig = auxPrimIdx % 3
    // We *still* need geometry for OptiX to hit. Let's define the auxiliary
    // triangles using the original vertices, making them co-planar. The robustness
    // relies ENTIRELY on the ray hitting the correct SBT record for the aux triangle.
    // This seems fragile. Let's assume the paper *does* imply new vertices.
    // This implementation becomes much larger as we need vertex buffer expansion.
    //
    // *** SIMPLIFICATION FOR NOW: ***
    // We will *not* implement the generation kernel yet. We will allocate the
    // aux_indices buffer but leave it empty or zeroed. The GAS build will use it.
    // This means the Mochi collision logic won't work correctly yet, but the
    // framework (build, destroy, interface) will be testable.

    // Placeholder: Fill with original indices for structure, knowing this is WRONG for Mochi logic.
     auxIndices[triIdx*3 + 0] = vIdx; // Aux 0 (edge 0)
     auxIndices[triIdx*3 + 1] = vIdx; // Aux 1 (edge 1)
     auxIndices[triIdx*3 + 2] = vIdx; // Aux 2 (edge 2)

}

extern "C" void mochiKernelDestroy_impl(MochiKernelData_impl* handle) {
    std::cout << "  mochiKernelDestroy_impl starting..." << std::endl;
    if (!handle){ return; }

    // Cast pointers to void* for cudaFree
    if (handle->d_vertices) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(handle->d_vertices)));
    if (handle->d_original_indices) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(handle->d_original_indices)));
    if (handle->d_aux_indices) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(handle->d_aux_indices)));
    if (handle->d_original_face_indices) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(handle->d_original_face_indices)));
    if (handle->d_gas_output_buffer) CUDA_CHECK(cudaFree(reinterpret_cast<void*>(handle->d_gas_output_buffer)));

    // ... (Reset pointers) ...
    delete handle;
    std::cout << "  mochiKernelDestroy_impl finished." << std::endl;
}

// ... (Rest of the file remains the same) ...
extern "C" void mochiKernelIntersectTriangle_impl(
    const MochiKernelData_impl* bvhHandle,
    const MochiTrianglePOD& inputTriangle,
    std::vector<int>& outCollidingTriangleIndices)
{
    outCollidingTriangleIndices.clear();
    if (!bvhHandle || !g_optixContext) {
         std::cerr << "Cannot intersect, kernel not built or OptiX not initialized." << std::endl;
         return;
    }
     std::cerr << "ERROR: mochiKernelIntersectTriangle_impl not fully implemented." << std::endl;
    // TODO: Implement the specific launch logic for single triangle intersection.
    // This would involve setting up Params struct with inputTriangle data,
    // launching only 3 rays in RayGen, and collecting results similarly to
    // the kernel-kernel intersection but simplified.
}

extern "C" void mochiKernelIntersect_impl(
    const MochiKernelData_impl* handleA,
    const MochiKernelData_impl* handleB,
    bool selfIntersection,
    std::vector<MochiCollisionPairPOD>& outCollisionPairs)
{

     outCollisionPairs.clear();
     if (!handleA || !handleB || !g_optixContext || !g_optixPipeline) {
         std::cerr << "Cannot intersect, kernels not built or OptiX not initialized/pipelined." << std::endl;
         return;
     }
     // std::cout << "mochiKernelIntersect_impl starting..." << std::endl;
     // std::cout << "  Kernel A Tris: " << handleA->numOriginalTriangles << ", Verts: " << handleA->numVertices << std::endl;
     // std::cout << "  Kernel B Tris: " << handleB->numOriginalTriangles << ", Verts: " << handleB->numVertices << std::endl;

     CUdeviceptr d_params = 0;
     CUdeviceptr d_collisionBuffer = 0;
     CUdeviceptr d_collisionCounter = 0;
     HitGroupData hgRecords[2]; // Host copy to update pointers

     try {
         // 1. Prepare Launch Parameters
         Params params = {};
         params.targetGas = handleB->gasHandle;
         params.meshAVertices = reinterpret_cast<Point_cu*>(handleA->d_vertices);
         params.meshAIndices = reinterpret_cast<int3*>(handleA->d_original_indices);
         params.meshANumOriginalTriangles = handleA->numOriginalTriangles;
         params.selfIntersectionCheck = selfIntersection;

         // 2. Allocate Output Buffer & Counter
         // Estimate max collisions - needs tuning! Consider a resize strategy if needed.
         unsigned int maxCollisions = (handleA->numOriginalTriangles / 5) + 1000; // Increased estimate
         params.maxCollisions = maxCollisions;

         CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_collisionBuffer), maxCollisions * sizeof(MochiCollisionPairPOD)));
         CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_collisionCounter), sizeof(unsigned int)));
         CUDA_CHECK(cudaMemset(reinterpret_cast<void*>(d_collisionCounter), 0, sizeof(unsigned int)));
         params.collisionBuffer = reinterpret_cast<MochiCollisionPairPOD*>(d_collisionBuffer);
         params.collisionCounter = reinterpret_cast<unsigned int*>(d_collisionCounter);

         // Copy params to device constant memory
         CUDA_CHECK(cudaMemcpyToSymbol(params, &params, sizeof(Params), 0, cudaMemcpyHostToDevice));


         // 3. Update SBT Hit Record Data Pointers dynamically before launch
         CUDA_CHECK(cudaMemcpy(hgRecords, reinterpret_cast<void*>(g_d_sbt_hitgroup_records), sizeof(HitGroupData) * 2, cudaMemcpyDeviceToHost)); // Read current SBT

         hgRecords[0].vertices = reinterpret_cast<Point_cu*>(handleB->d_vertices);
         hgRecords[0].indices = reinterpret_cast<int3*>(handleB->d_original_indices);
         hgRecords[0].originalFaceIndices = reinterpret_cast<int*>(handleB->d_original_face_indices);
         hgRecords[0].numOriginalTriangles = handleB->numOriginalTriangles;

         hgRecords[1].vertices = reinterpret_cast<Point_cu*>(handleB->d_vertices);
         hgRecords[1].indices = reinterpret_cast<int3*>(handleB->d_aux_indices);
         hgRecords[1].originalFaceIndices = nullptr;
         hgRecords[1].numOriginalTriangles = handleB->numOriginalTriangles;

         CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(g_d_sbt_hitgroup_records), hgRecords, sizeof(HitGroupData) * 2, cudaMemcpyHostToDevice)); // Write updated SBT


         // 4. Launch OptiX
         if (handleA->numOriginalTriangles > 0) {
           // std::cout << "  Launching OptiX (" << handleA->numOriginalTriangles << "x3)..." << std::endl;
            OPTIX_CHECK(optixLaunch(
                g_optixPipeline,               // The pipeline to use
                0,                             // CUDA stream (0 for default)
                d_params,                      // Pointer to launch parameters on device
                sizeof(Params),                // Size of the launch parameters
                &g_sbt,                        // Pointer to the Shader Binding Table
                handleA->numOriginalTriangles, // Launch dimension X (one thread per source triangle)
                3,                             // Launch dimension Y (one thread per edge of the source triangle)
                1                              // Launch dimension Z
            ));
           // OPTIX_CHECK(optixLaunch(
           //       g_optixPipeline,
           //       0, // stream
           //       reinterpret_cast<CUdeviceptr>(0), // Use constant memory for params
           //       0,                                // Size is 0 when using constant memory symbol
           //       &g_sbt,
           //       handleA->numOriginalTriangles, // launch width
           //       3,                             // launch height
           //       1                              // launch depth
           //   ));
             CUDA_CHECK(cudaDeviceSynchronize());
             // std::cout << "  OptiX Launch finished." << std::endl;
         } else {
           // std::cout << "  Skipping OptiX Launch (0 source triangles)." << std::endl;
         }

         // 5. Copy Results Back
         unsigned int numCollisionsFound = 0;
         CUDA_CHECK(cudaMemcpy(&numCollisionsFound, reinterpret_cast<const void*>(d_collisionCounter), sizeof(unsigned int), cudaMemcpyDeviceToHost));
         // std::cout << "  Collisions found on GPU: " << numCollisionsFound << std::endl;

         if (numCollisionsFound > 0) {
             if (numCollisionsFound > maxCollisions) {
                  std::cerr << "Warning: Collision buffer overflowed! Found " << numCollisionsFound << ", allocated for " << maxCollisions << ". Truncating results." << std::endl;
                  numCollisionsFound = maxCollisions;
             }
             outCollisionPairs.resize(numCollisionsFound);
             CUDA_CHECK(cudaMemcpy(outCollisionPairs.data(), reinterpret_cast<const void*>(d_collisionBuffer), numCollisionsFound * sizeof(MochiCollisionPairPOD), cudaMemcpyDeviceToHost));
         }

         // 6. Cleanup temporary buffers
         CUDA_CHECK(cudaFree(reinterpret_cast<void*>(d_collisionBuffer)));
         CUDA_CHECK(cudaFree(reinterpret_cast<void*>(d_collisionCounter)));
         // cudaFree(d_params); // d_params was not allocated with cudaMalloc

     } catch (const std::exception& e) {
          std::cerr << "Exception during MochiKernel intersection: " << e.what() << std::endl;
          if (d_collisionBuffer) cudaFree(reinterpret_cast<void*>(d_collisionBuffer));
          if (d_collisionCounter) cudaFree(reinterpret_cast<void*>(d_collisionCounter));
          // if (d_params) cudaFree(reinterpret_cast<void*>(d_params)); // Should not be freed if cudaMemcpyToSymbol was used
          throw;
     }
}
