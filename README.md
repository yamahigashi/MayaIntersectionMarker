# Maya Intersection Marker

A plugin for Autodesk Maya that detects and visualizes mesh intersections in the viewport.
![Sample](https://github.com/yamahigashi/MayaIntersectionMarker/blob/doc/doc/Animation.gif)


## Description

The Maya Intersection Marker is a plugin designed to streamline the process of checking for 
mesh intersections or 'penetrations' in animations. By automatically detecting and visualizing 
these intersections within Maya's viewport, this plugin reduces the time and effort required 
to identify and resolve these issues, ensuring smoother animations and improved efficiency in 
your workflow.

## Features

* Automatic detection of mesh intersections
* Real-time visualization of intersections in the viewport
* Supports complex mesh geometries
* User-friendly interface

## Installation
Follow the steps below to install the Maya Intersection Marker plugin:

1. **Download the ZIP File:** Navigate to the [Releases](https://github.com/yamahigashi/MayaIntersectionMarker/releases) page of this repository and download the latest [ZIP file](https://github.com/yamahigashi/MayaIntersectionMarker/releases/download/1.0.0/MayaIntersectionMarker.zip).

2. **Extract the ZIP File:** Once downloaded, extract the contents of the ZIP file.

3. **Move Files to Maya Modules Folder:** Take the extracted files and move them to the Maya modules folder, typically located at `%USERPROFILE%\Documents\maya\modules`.

Note: `%USERPROFILE%` is an environment variable that corresponds to the current user's home directory. Make sure to check if your Maya modules folder is located in this directory.

Additionally, the Maya modules folder can be any location that has been configured in the `MAYA_MODULE_PATH` environment variable. Ensure that this environment variable is properly set if you wish to use a custom location for your Maya modules.

## Compatibility
- Maya 2024 windows
- Maya 2023 windows
- Maya 2022 windows
- Maya 2020 windows


## Usage
Using the Maya Intersection Marker plugin is straightforward:

1. **Select Two Meshes:** In the Maya scene, select two meshes that you want to check for intersections.

2. **Run the Command:** After selecting the meshes, execute the menu `Window > Intersection Marker`, or run the `intersectionMarker` command in the command line or script editor.

This will visualize any intersections between the selected meshes directly within the viewport.



## Build Instructions

This section is for developers who want to contribute to the project or build it from source. Here's how you can build the Maya Intersection Marker plugin:

### Prerequisites

* **CMake:** Used for build automation. You can download and install CMake from [here](https://cmake.org/download/).
* **Embree:** A high-performance ray tracing kernel library. You can download and install Embree from [here](https://github.com/embree/embree/releases).
* **GLM:** A header-only C++ mathematics library for graphics software. You can download and install GLM from [here](https://github.com/g-truc/glm/releases).
* **Autodesk Maya SDK:** Make sure you have the SDK installed for the version of Maya you are developing for from [here](https://aps.autodesk.com/developer/overview/maya)


### Build Steps

1. **Clone the Repository:** Clone this repository to your local machine.
2. **Navigate to Project Directory:** Use your terminal to navigate to the project directory.
3. **Edit `build.bat`:** Edit the `build.bat` script to match your specific environment setup. Make sure to adjust the settings to match the location of your Autodesk Maya SDK and other necessary configurations.
4. **Run `build.bat`:** Run the `build.bat` script to create a build directory, configure the project with CMake, and build the project. 

### Troubleshooting

If you encounter any issues while building the project, please open an issue in this repository or refer to the CMake or Embree documentation.

### Additional Notes

Make sure that your system's environment variables are set up correctly for both CMake and Embree. Proper paths should be included in your system's PATH variable.


## License
MIT

## Contributing

This project is in its early stages, and we warmly welcome any feedback or contributions. Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. Ensure the tests pass before you make a request.

## Contact

yamahigashi - [anamorphobia@𝕏Twitter](https://twitter.com/anamorphobia)

Project Link: [https://github.com/yamahigashi/MayaIntersectionMarker](https://github.com/yamahigashi/MayaIntersectionMarker)


---

## 概要
Maya Intersection Markerは、アニメーション中のメッシュの交差や「貫通」をチェックする
プラグインです。Mayaのビューポート内でメッシュどうしの交差をリアルタイムに検出し、可視化
することが可能です。目視によるチェックの労力を削減し、ワークフローの効率向上を実現します。

## 特徴
- メッシュの交差の自動検出
- ビューポートでの交差のリアルタイム可視化
- 複雑なメッシュジオメトリのサポート


## インストール
Maya Intersection Markerプラグインをインストールする手順は以下の通りです：

1. ZIPファイルのダウンロード: このリポジトリの[Releases](https://github.com/yamahigashi/MayaIntersectionMarker/releases)ページに移動し、最新の[ZIP file](https://github.com/yamahigashi/MayaIntersectionMarker/releases/download/1.0.0/MayaIntersectionMarker.zip)をダウンロードします。
2. ZIPファイルの展開: ダウンロードしたZIPファイルの内容を展開します。
3. ファイルのMayaモジュールフォルダへの移動: 展開したファイルをMayaのモジュールフォルダに移動します。通常、このフォルダは `%USERPROFILE%\Documents\maya\modules` にあります。

注意: %USERPROFILE%は、現在のユーザーのホームディレクトリに対応する環境変数です。Mayaのモジュールフォルダがこのディレクトリにあることを確認してください。

また、Mayaのモジュールをカスタムの場所で使用する場合は、MAYA_MODULE_PATH環境変数で正しく設定されていることを確認してください。


## サポート環境
- Maya 2024 windows
- Maya 2023 windows
- Maya 2022 windows
- Maya 2020 windows

他の環境での動作が必要なばあい、ビルド手順を参照しビルドを行ってください。

## 使用方法

Maya Intersection Markerプラグインの使用は簡単です：

1. 2つのメッシュを選択: Mayaのシーンで、交差をチェックしたい2つのメッシュを選択します。

2. コマンドを実行: メッシュを選択した後、メニュー `Window > Intersection Marker` を実行するか、コマンドラインまたはスクリプトエディタで `intersectionMarker` コマンドを実行します。


## ビルド手順

このセクションは、プロジェクトに貢献したり、ソースからプロジェクトをビルドしたい開発者向けです。Maya Intersection Markerプラグインをビルドする方法は次のとおりです：


### 事前に必要なもの

* CMake: ビルド自動化に使用します。CMakeはここからダウンロードしてインストールできます。
* Embree: 高性能なレイトレーシングカーネルライブラリです。Embreeはここからダウンロードしてインストールできます。
* GLM: グラフィックソフトウェア向けのヘッダーオンリーのC++数学ライブラリです。GLMはここからダウンロードしてインストールできます。
* Autodesk Maya SDK: 開発対象のMayaバージョンのSDKをここからインストールしてください。


### ビルド手順

1. リポジトリのクローン: このリポジトリをローカルマシンにクローンします。
2. プロジェクトディレクトリへの移動: ターミナルを使用してプロジェクトディレクトリに移動します。
3. build.batの編集: build.batスクリプトを編集して、特定の環境設定に合わせます。Autodesk Maya SDKの場所やその他の必要な設定を調整してください。
4. build.batの実行: build.batスクリプトを実行してビルドディレクトリを作成し、CMakeでプロジェクトを構成し、プロジェクトをビルドします。
