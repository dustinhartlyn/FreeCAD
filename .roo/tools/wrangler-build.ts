import { parametersSchema as z, defineCustomTool, CustomToolContext } from "@roo-code/types"
//@ts-ignore spawnSync really does exist
import { spawnSync } from "child_process"

// Wrapping with vcvarsall.bat x64 is required on this workspace: raw pixi/cmake
// execution drops standard library include paths (<set>, <memory>, etc.) and
// breaks compilation. See Agent_Notes/.wrangler_runbook.md.
const VCVARSALL = String.raw`D:\PROGRA~2\MICROS~1\18\COMMUN~1\VC\Auxiliary\Build\vcvarsall.bat`

export default defineCustomTool({
	name: "wrangler_build",
	description:
		"[v1] Deterministic build dispatcher. If any changed file is a header (.h/.hpp), runs a full workspace rebuild (pixi run build). Otherwise runs a targeted incremental cmake build for the given target. Always wraps with vcvarsall.bat x64.",
	parameters: z.object({
		changedFiles: z.array(z.string()).describe("File paths changed in this task"),
		target: z.string().default("Sketcher").describe("CMake target for a targeted build; ignored if a header changed"),
	}),
	async execute({ changedFiles, target }, context: CustomToolContext) {
		//@ts-ignore cwd really does exist
		const basePath = context.task.cwd
		const headerChanged = changedFiles.some((f: string) => /\.(h|hpp)$/i.test(f))
		const inner = headerChanged ? "pixi run build" : `cmake --build build\\debug --target ${target}`
		const cmd = `call "${VCVARSALL}" x64 && ${inner}`

		//@ts-ignore say exists
		context.task.say(
			"custom_tool",
			headerChanged
				? `Header change detected (${changedFiles.join(", ")}) -> full workspace rebuild\n${cmd}`
				: `No header changes -> targeted build (${target})\n${cmd}`
		)

		const result = spawnSync(cmd, { cwd: basePath, shell: true, encoding: "utf-8" })
		if (result.status === 0) {
			return `Success (${headerChanged ? "full rebuild" : "targeted: " + target})`
		}
		return `Failed with code ${result.status}\n${tail(result.stdout)}\n${tail(result.stderr)}`
	},
})

function tail(text: string | null | undefined, numLines = 200): string {
	if (!text) return ""
	return text.trim().split("\n").slice(-numLines).join("\n")
}
