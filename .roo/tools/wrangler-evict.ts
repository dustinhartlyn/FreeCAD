import { parametersSchema as z, defineCustomTool, CustomToolContext } from "@roo-code/types"
//@ts-ignore spawnSync really does exist
import { spawnSync } from "child_process"
//@ts-ignore fs really does exist
import * as fs from "fs"
//@ts-ignore path really does exist
import * as path from "path"

export default defineCustomTool({
	name: "wrangler_evict",
	description:
		"[v1] Evicts a locked process (default FreeCAD) after logging the action and reason to Agent_Notes/.wrangler_runbook.md FIRST, so the action is recorded even if eviction has side effects (e.g. killing a session with unsaved user work). Only use for known build/runtime lock-holders, not arbitrary process names.",
	parameters: z.object({
		processName: z.string().default("FreeCAD").describe("Process name to evict"),
		reason: z.string().describe("Why this eviction is being run, e.g. the exact compiler error that triggered it"),
	}),
	async execute({ processName, reason }, context: CustomToolContext) {
		//@ts-ignore cwd really does exist
		const basePath = context.task.cwd
		const timestamp = new Date().toISOString()
		const logLine = `- [${timestamp}] OS HANDLE EVICTION: Stop-Process -Name ${processName} -Force. Reason: ${reason}\n`
		const logPath = path.join(basePath, "Agent_Notes", ".wrangler_runbook.md")
		fs.appendFileSync(logPath, logLine, "utf-8")

		//@ts-ignore say exists
		context.task.say("custom_tool", `Logged eviction to ${logPath}. Evicting ${processName} ...`)

		const result = spawnSync(
			`powershell -NoProfile -Command "Stop-Process -Name '${processName}' -Force -ErrorAction SilentlyContinue"`,
			{ cwd: basePath, shell: true, encoding: "utf-8" }
		)
		return `Logged to Agent_Notes/.wrangler_runbook.md and evicted ${processName} (exit ${result.status}).`
	},
})
