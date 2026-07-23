/**
 * Shared cross-tab action queue for Control | Movement | Calibration.
 *
 * Motor execution always runs from the Movement page via the sequence API.
 * Other tabs enqueue validated action dicts into localStorage.
 */
(function (global) {
  const STORAGE_KEY = 'cat_follow_action_queue_v1';
  const EVENT_NAME = 'cat-follow-queue-changed';

  function readQueue() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return [];
      const parsed = JSON.parse(raw);
      return Array.isArray(parsed) ? parsed : [];
    } catch (_e) {
      return [];
    }
  }

  function writeQueue(actions) {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(actions));
    global.dispatchEvent(new CustomEvent(EVENT_NAME, { detail: actions }));
  }

  function actionSummary(action) {
    if (!action || !action.type) return '(invalid)';
    if (action.type === 'drive') {
      return `drive ${action.direction} @ ${action.speed_pct}% for ${action.duration_s}s`;
    }
    if (action.type === 'steer') {
      return `steer ${action.angle_deg}° @ ${action.speed_pct}% for ${action.duration_s}s`;
    }
    if (action.type === 'wait') {
      return `wait ${action.duration_s}s`;
    }
    if (action.type === 'stop') {
      return `stop ${action.duration_s || 0.5}s`;
    }
    return action.type;
  }

  function defaultAction(type) {
    if (type === 'drive') {
      return { type: 'drive', direction: 'forward', speed_pct: 30, duration_s: 1.0 };
    }
    if (type === 'steer') {
      return { type: 'steer', angle_deg: 15, speed_pct: 30, duration_s: 2.0 };
    }
    if (type === 'wait') {
      return { type: 'wait', duration_s: 1.0 };
    }
    return { type: 'stop', duration_s: 0.5 };
  }

  function authHeaders(extra) {
    const headers = Object.assign({ 'Content-Type': 'application/json' }, extra || {});
    const token = sessionStorage.getItem('cat_follow_control_token')
      || localStorage.getItem('cat_follow_control_token')
      || '';
    if (token) headers['X-Control-Token'] = token;
    return headers;
  }

  function rememberToken(token) {
    if (!token) return;
    sessionStorage.setItem('cat_follow_control_token', token);
  }

  function ensureToken() {
    let token = sessionStorage.getItem('cat_follow_control_token')
      || localStorage.getItem('cat_follow_control_token')
      || '';
    if (token) return token;
    token = prompt('Enter control token (leave blank if unset on server):') || '';
    if (token) rememberToken(token);
    return token;
  }

  global.CatFollowActionBuilder = {
    STORAGE_KEY,
    EVENT_NAME,
    readQueue,
    writeQueue,
    actionSummary,
    defaultAction,
    authHeaders,
    rememberToken,
    ensureToken,

    addActions(actions) {
      const queue = readQueue();
      queue.push(...actions);
      writeQueue(queue);
      return queue;
    },

    removeAt(index) {
      const queue = readQueue();
      queue.splice(index, 1);
      writeQueue(queue);
      return queue;
    },

    moveUp(index) {
      const queue = readQueue();
      if (index <= 0 || index >= queue.length) return queue;
      const tmp = queue[index - 1];
      queue[index - 1] = queue[index];
      queue[index] = tmp;
      writeQueue(queue);
      return queue;
    },

    clearQueue() {
      writeQueue([]);
      return [];
    },

    replaceQueue(actions) {
      writeQueue(actions.slice());
      return readQueue();
    },

    mixin(extra) {
      return Object.assign({
        queue: [],
        draftType: 'drive',
        draft: defaultAction('drive'),
        sequenceStatus: {},
        heartbeatTimer: null,
        statusTimer: null,
        builderFeedback: '',
        builderFeedbackOk: true,

        initActionBuilder() {
          this.refreshQueue();
          global.addEventListener(EVENT_NAME, () => this.refreshQueue());
          global.addEventListener('storage', (ev) => {
            if (ev.key === STORAGE_KEY) this.refreshQueue();
          });
        },

        refreshQueue() {
          this.queue = readQueue();
        },

        setDraftType(type) {
          this.draftType = type;
          this.draft = defaultAction(type);
        },

        addDraftToQueue() {
          this.queue = this.addActions([Object.assign({}, this.draft)]);
          this.showBuilderFeedback('Added to queue', true);
        },

        addActionsToQueue(actions) {
          this.queue = this.addActions(actions);
          this.showBuilderFeedback(`Added ${actions.length} action(s) to queue`, true);
        },

        removeQueueItem(index) {
          this.queue = this.removeAt(index);
        },

        moveQueueItemUp(index) {
          this.queue = this.moveUp(index);
        },

        clearActionQueue() {
          this.queue = this.clearQueue();
        },

        showBuilderFeedback(msg, ok) {
          this.builderFeedback = msg;
          this.builderFeedbackOk = ok;
          setTimeout(() => { this.builderFeedback = ''; }, 3500);
        },

        async validateQueue() {
          ensureToken();
          const res = await fetch('/api/movement/sequence/validate', {
            method: 'POST',
            headers: authHeaders(),
            body: JSON.stringify({ actions: this.queue }),
          });
          const data = await res.json();
          if (!res.ok) {
            throw new Error((data.errors && data.errors.join('; ')) || data.error || 'Validation failed');
          }
          return data.actions || this.queue;
        },

        async runActionQueue() {
          try {
            ensureToken();
            const res = await fetch('/api/movement/sequence/run', {
              method: 'POST',
              headers: authHeaders(),
              body: JSON.stringify({ actions: this.queue }),
            });
            const data = await res.json();
            if (!res.ok) {
              throw new Error(data.error || (data.errors && data.errors.join('; ')) || 'Run failed');
            }
            this.startSequenceHeartbeat();
            this.startSequenceStatusPoll();
            this.showBuilderFeedback(`Sequence started (${data.mode})`, true);
            return data;
          } catch (err) {
            this.showBuilderFeedback(err.message || 'Run failed', false);
            throw err;
          }
        },

        async stopSequence() {
          const res = await fetch('/api/movement/sequence/stop', { method: 'POST' });
          const data = await res.json().catch(() => ({}));
          this.stopSequenceHeartbeat();
          if (res.ok) {
            this.showBuilderFeedback('Sequence stopped', true);
          } else {
            this.showBuilderFeedback(data.error || 'Stop failed', false);
          }
          await this.fetchSequenceStatus();
        },

        startSequenceHeartbeat() {
          this.stopSequenceHeartbeat();
          this.heartbeatTimer = setInterval(async () => {
            try {
              await fetch('/api/movement/sequence/heartbeat', {
                method: 'POST',
                headers: authHeaders(),
              });
            } catch (_e) { /* ignore */ }
          }, 200);
        },

        stopSequenceHeartbeat() {
          if (this.heartbeatTimer) {
            clearInterval(this.heartbeatTimer);
            this.heartbeatTimer = null;
          }
        },

        startSequenceStatusPoll() {
          if (this.statusTimer) return;
          this.statusTimer = setInterval(() => this.fetchSequenceStatus(), 500);
          this.fetchSequenceStatus();
        },

        async fetchSequenceStatus() {
          try {
            const res = await fetch('/api/movement/sequence/status');
            if (!res.ok) return;
            this.sequenceStatus = await res.json();
            const status = this.sequenceStatus.status;
            if (status === 'completed' || status === 'aborted' || status === 'idle') {
              this.stopSequenceHeartbeat();
            }
          } catch (_e) { /* ignore */ }
        },

        queueSummary(action) {
          return actionSummary(action);
        },
      }, extra || {});
    },
  };
})(window);
