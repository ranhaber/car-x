# Synthetic hunk snippets used by smoke_test.py (documentation only).
# Executable coverage lives in ../smoke_test.py.

## 1. Comment-only → shallow
```diff
@@ -12,3 +12,4 @@
 # existing
+# clarify helper behavior
 def helper():
```

## 2. Lock / shared state → deep + C1
```diff
@@ -40,3 +40,5 @@
 def update():
+    with self._lock:
+        self.value += 1
     return self.value
```

## 3. Frame ring → deep + C2 + Frame_Ring doc
```diff
@@ -20,3 +20,4 @@
 def acquire():
+    lease = self._ring.lease(generation=self._gen)
     return buf
```

## 4. ROS bridge → deep + ROS2
```diff
@@ -55,3 +55,4 @@
 def setup(self):
+    self.create_subscription(Pose, "/odom", self._cb, 10)
     return
```
