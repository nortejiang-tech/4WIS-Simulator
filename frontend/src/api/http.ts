// Shared fetch helper — JSON in/out, timeout via AbortController, and
// error messages that surface the backend's `detail` field (FastAPI 4xx).

export class HttpError extends Error {
  status: number;
  constructor(status: number, message: string) {
    super(message);
    this.status = status;
  }
}

const DEFAULT_TIMEOUT_MS = 8000;

export async function fetchJSON<T = unknown>(
  url: string,
  init?: RequestInit,
  timeoutMs: number = DEFAULT_TIMEOUT_MS,
): Promise<T> {
  const ctrl = new AbortController();
  const timer = window.setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    const r = await fetch(url, { ...init, signal: ctrl.signal });
    if (!r.ok) {
      let detail = `${r.status}`;
      try {
        const body = await r.json();
        if (body?.detail) {
          detail = typeof body.detail === "string" ? body.detail : JSON.stringify(body.detail);
        }
      } catch {
        /* non-JSON error body */
      }
      throw new HttpError(r.status, detail);
    }
    // 204 / empty bodies
    const text = await r.text();
    return (text ? JSON.parse(text) : undefined) as T;
  } catch (e) {
    if (e instanceof DOMException && e.name === "AbortError") {
      throw new HttpError(0, `请求超时（${Math.round(timeoutMs / 1000)}s）：${url}`);
    }
    throw e;
  } finally {
    window.clearTimeout(timer);
  }
}

export function postJSON<T = unknown>(url: string, body?: unknown, timeoutMs?: number): Promise<T> {
  return fetchJSON<T>(
    url,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body !== undefined ? JSON.stringify(body) : undefined,
    },
    timeoutMs,
  );
}

export function patchJSON<T = unknown>(url: string, body?: unknown, timeoutMs?: number): Promise<T> {
  return fetchJSON<T>(
    url,
    {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: body !== undefined ? JSON.stringify(body) : undefined,
    },
    timeoutMs,
  );
}

export function deleteJSON<T = unknown>(url: string, timeoutMs?: number): Promise<T> {
  return fetchJSON<T>(url, { method: "DELETE" }, timeoutMs);
}
