export function canDriveInput(context: { page: string; armed: boolean; online: boolean; visible: boolean; focused: boolean; editing: boolean; paused: boolean; scripted: boolean }): boolean {
  return context.page === "run" && context.armed && context.online && context.visible && context.focused && !context.editing && !context.paused && !context.scripted;
}

export function isEditingTarget(element: Element | null): boolean {
  return !!element?.closest("input, textarea, select, [contenteditable]:not([contenteditable=false]), [role=textbox]");
}
