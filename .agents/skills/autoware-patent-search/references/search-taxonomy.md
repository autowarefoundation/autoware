# Autoware patent-search taxonomy

Use these terms to build cross-database patent queries. Mix broad domain terms with the changed algorithm's specific terms.

## Target databases

Only use these patent databases unless the user explicitly changes scope later:

- **Google Patents**: useful for international full-text searching, English/Japanese terms, CPC hints, and assignee filters.
- **J-PlatPat**: useful for Japan-focused searches, FI/F-term exploration, Japanese terminology, and Japanese assignee names. Prefer recording the keywords and filters used because stable deep links may not always be available.

## English terms

- autonomous vehicle, automated driving, self-driving vehicle, ADAS
- behavior planning, behavior planner, maneuver planning, decision making
- trajectory planning, path planning, motion planning, route planning
- vehicle control, lateral control, longitudinal control, trajectory tracking
- lane change, lane keeping, lane departure, obstacle avoidance, path generation
- drivable area, occupancy grid, cost map, predicted path, object prediction
- collision checking, time-to-collision, risk assessment, safety margin
- traffic signal recognition, stop line, intersection handling, right of way
- pedestrian interaction, vulnerable road user, cut-in vehicle, following vehicle
- minimum risk maneuver, MRM, fallback, fail operational, emergency stop, degradation
- HD map, lanelet, road boundary, map matching, localization uncertainty
- V2X, infrastructure, cooperative driving, connected vehicle

## Japanese terms

- 自動運転, 自律走行, 運転支援, 先進運転支援, ADAS
- 行動計画, 経路計画, 軌道計画, 走行計画, 動作計画
- 車両制御, 横制御, 縦制御, 軌道追従
- 車線変更, 車線維持, 障害物回避, 経路生成
- 走行可能領域, 占有格子, コストマップ, 予測経路, 物体予測
- 衝突判定, 衝突回避, 衝突リスク, 安全余裕, 車間距離
- 信号機, 停止線, 交差点, 優先権, 右左折
- 歩行者, 交通参加者, 割り込み車両, 後続車
- 最小リスク操作, MRM, フォールバック, フェイルオペレーショナル, 緊急停止, 縮退運転
- 高精度地図, レーンレット, 道路境界, マップマッチング, 自己位置推定誤差
- 路車間通信, V2X, インフラ協調, 協調走行

## Company/assignee search terms

Use these only when the user asks to review a particular company's portfolio or wants candidate assignees surfaced for their own review.

- English examples: Toyota, Honda, Nissan, Denso, Aisin, Hitachi Astemo, Bosch, Continental, Mobileye, Waymo, Aptiv, ZF
- Japanese examples: トヨタ, 本田技研, ホンダ, 日産, デンソー, アイシン, 日立Astemo, ボッシュ, コンチネンタル

## CPC/IPC and Japan classification hints

Use these only as search hints:

- B60W: vehicle control systems for different vehicle sub-units, ADAS, automated driving control
- G05D1/00: control of position, course, altitude, or attitude of vehicles
- G08G1/00: traffic control systems, road vehicle traffic management
- B62D15/00: steering control and path following
- B60T7/00 or B60T8/00: braking control and vehicle safety braking
- G06V20/56: image/video understanding for autonomous driving scenes
- G01C21/00: navigation and route guidance
- H04W4/40: vehicle communication services, V2X-related communication
- J-PlatPat FI/F-term exploration is useful after a relevant Japanese document is found; record FI/F-term values if they appear in candidate results.
