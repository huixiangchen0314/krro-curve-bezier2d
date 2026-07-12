(ns top.kzre.krro.curve.bezier2d.spec
  "Bézier 曲线 EDN 数据结构的规格定义。"
  (:require [clojure.spec.alpha :as s]))

;; ── 点 ──────────────────────────────────────────────
(s/def ::x number?)
(s/def ::y number?)
(s/def ::point (s/keys :req-un [::x ::y]))

;; ── 控制点 ──────────────────────────────────────────
(s/def ::dx1 number?)
(s/def ::dy1 number?)
(s/def ::dx2 number?)
(s/def ::dy2 number?)
(s/def ::g1 boolean?)
(s/def ::control-point (s/merge ::point (s/keys :req-un [::dx1 ::dy1 ::dx2 ::dy2 ::g1])))

;; ── 曲线（路径） ───────────────────────────────────
(s/def ::closed boolean?)
(s/def ::points (s/coll-of ::control-point :kind vector? :min-count 2))
(s/def ::curve (s/keys :req-un [::closed ::points]))

;; ── 包围盒 ──────────────────────────────────────────
(s/def ::min-x number?)
(s/def ::min-y number?)
(s/def ::max-x number?)
(s/def ::max-y number?)
(s/def ::aabb (s/keys :req-un [::min-x ::min-y ::max-x ::max-y]))

;; ── 最近点结果 ──────────────────────────────────────
(s/def ::t number?)
(s/def ::distance (s/and number? (complement neg?)))
(s/def ::closest-result (s/keys :req-un [::point ::t ::distance]))