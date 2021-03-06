﻿///////////////////////////////////////////////////////////////////////////////
// Easy Physics SDK パッケージ Version 0.8

===============================================================================
== 概要

Easy Physics SDKはリアルタイム3D剛体シミュレーションを実現するためのライブラリで
す。剛体を扱うための基本的な処理に加え、様々なジョイントや、連続的に物理を扱うた
めの仕組みを提供します。

インプレスジャパン社より出版されている「ゲーム制作者のための物理シミュレーション
　剛体編」に、「Easy Physics」の基本的なアルゴリズムの詳細や扱い方が解説されてい
ます。本ライブラリは、「Easy Physics」にいくつかの改修を施したバージョンとなって
います。

■主な改修点
　- Speculative Contactsによるコリジョン抜けを防ぐ仕組みの導入
　- 様々なジョイントを6DOFジョイントで表現
　- 1軸Sort and sweepでブロードフェーズを高速化
　- 凸メッシュ同士の衝突検出を高速化

===============================================================================
== 機能

剛体は全て凸メッシュ（Convex Hull）として扱います。
各剛体は最大５つまでの凸メッシュを保持します。
ジョイントは以下の6種をサポートします。
 - ボールジョイント、ヒンジジョイント、固定ジョイント、スイングツイストジョイン
ト、スライダジョイント

===============================================================================
== セットアップ

・ライブラリ用Visual Studioプロジェクトファイル
EasyPhysicsSdk/easy_physics/project/easy_physics.vcxproj

・サンプル用Visual Studioソリューションファイル
EasyPhysicsSdk/sample/physics_sample_vs2010.sln

===============================================================================
== サンプルについて

全てのサンプルはphysics_func.cppのphysicsCreateScene()でシーンを作成、
physicsSimulate()でシミュレーションステップを実行します。

■EasyPhysicsSdk/sample/01_basic
基本的な剛体シミュレーションの機能を紹介するサンプルです。

■EasyPhysicsSdk/sample/02_compound
複合形状や重心をずらした剛体など、応用的な使い方を紹介するサンプルです。

■EasyPhysicsSdk/sample/03_joint
剛体をジョイントで連結させ、複雑な機構を再現する紹介するサンプルです。

===============================================================================
== アプリケーションへの組み込み

・Easy Physics SDK APIの使用
アプリケーションプロジェクトにEasy Physics SDKのプロジェクトを追加し、コードに
「EasyPhysicsSdk/easy_physics/EpxInclude.h」をインクルードしてください。他の関連
ヘッダはEpxInclude.hから相対的に参照されます。

===============================================================================
== ビルド環境

OS : Windows XP以降
CPU : 1GHz以上
VGA : OpenGL対応ビデオカード
Compiler : Visual Studio 2010 (Express Edition可)

===============================================================================
https://github.com/hmatsuike/EasyPhysicsSdk
Hiroshi Matsuike(hmatsuike@gmail.com)
